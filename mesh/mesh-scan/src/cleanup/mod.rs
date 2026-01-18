//! Scan cleanup operations for meshes and point clouds.
//!
//! This module provides tools for cleaning up noisy scan data:
//! - **Outlier removal** - Remove statistically distant points
//! - **Spike removal** - Smooth or remove vertices that protrude abnormally
//! - **Unified cleanup** - One-step cleanup pipeline
//!
//! # Quick Start
//!
//! ```
//! use mesh_scan::cleanup::{cleanup_scan, CleanupParams};
//! use mesh_types::{IndexedMesh, Vertex};
//!
//! let mut mesh = IndexedMesh::new();
//! // ... load or create mesh ...
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! let params = CleanupParams::for_body_scan();
//! let result = cleanup_scan(&mesh, &params).unwrap();
//!
//! println!("{}", result);
//! ```
//!
//! # Workflow
//!
//! The cleanup pipeline runs these steps in order:
//! 1. Remove outlier vertices (if enabled)
//! 2. Remove spikes (if enabled)
//! 3. Remove small connected components (if enabled)
//! 4. Fill small holes (if enabled)

pub mod outlier;
pub mod spike;

pub use outlier::{
    remove_mesh_outliers, remove_outliers, remove_outliers_with_result, OutlierParams,
    OutlierRemovalResult,
};
pub use spike::{remove_spikes, SpikeParams, SpikeRemovalResult};

use mesh_repair::{find_connected_components, keep_largest_component};
use mesh_types::IndexedMesh;

use crate::error::{ScanError, ScanResult};

/// Parameters for the complete scan cleanup pipeline.
#[derive(Debug, Clone)]
pub struct CleanupParams {
    /// Whether to remove outlier vertices. Default: true.
    pub remove_outliers: bool,

    /// Parameters for outlier removal.
    pub outlier_params: OutlierParams,

    /// Whether to remove or smooth spikes. Default: true.
    pub remove_spikes: bool,

    /// Parameters for spike removal.
    pub spike_params: SpikeParams,

    /// Whether to remove small disconnected components. Default: true.
    pub remove_small_components: bool,

    /// Minimum number of faces for a component to be kept. Default: 100.
    pub min_component_faces: usize,

    /// Whether to fill small holes. Default: false.
    pub fill_holes: bool,

    /// Maximum number of edges for a hole to be filled. Default: 50.
    pub max_hole_edges: usize,
}

impl Default for CleanupParams {
    fn default() -> Self {
        Self {
            remove_outliers: true,
            outlier_params: OutlierParams::default(),
            remove_spikes: true,
            spike_params: SpikeParams::default(),
            remove_small_components: true,
            min_component_faces: 100,
            fill_holes: false,
            max_hole_edges: 50,
        }
    }
}

impl CleanupParams {
    /// Creates new parameters with defaults.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates parameters optimized for body scans.
    ///
    /// Body scans typically have:
    /// - Moderate noise
    /// - Large connected regions
    /// - Some small holes
    #[must_use]
    pub fn for_body_scan() -> Self {
        Self {
            remove_outliers: true,
            outlier_params: OutlierParams::new()
                .with_k_neighbors(25)
                .with_std_multiplier(2.5),
            remove_spikes: true,
            spike_params: SpikeParams::default(),
            remove_small_components: true,
            min_component_faces: 500,
            fill_holes: true,
            max_hole_edges: 100,
        }
    }

    /// Creates parameters optimized for object scans.
    ///
    /// Object scans typically have:
    /// - More noise on edges
    /// - Smaller features to preserve
    /// - Multiple disconnected parts possible
    #[must_use]
    pub fn for_object_scan() -> Self {
        Self {
            remove_outliers: true,
            outlier_params: OutlierParams::new()
                .with_k_neighbors(20)
                .with_std_multiplier(2.0),
            remove_spikes: true,
            spike_params: SpikeParams::conservative(),
            remove_small_components: true,
            min_component_faces: 50,
            fill_holes: false,
            max_hole_edges: 30,
        }
    }

    /// Creates minimal cleanup parameters.
    ///
    /// Only removes obvious outliers, preserves most geometry.
    #[must_use]
    pub fn minimal() -> Self {
        Self {
            remove_outliers: true,
            outlier_params: OutlierParams::conservative(),
            remove_spikes: false,
            spike_params: SpikeParams::default(),
            remove_small_components: false,
            min_component_faces: 100,
            fill_holes: false,
            max_hole_edges: 50,
        }
    }

    /// Creates aggressive cleanup parameters.
    ///
    /// Aggressively removes noise and small features.
    #[must_use]
    pub fn aggressive() -> Self {
        Self {
            remove_outliers: true,
            outlier_params: OutlierParams::aggressive(),
            remove_spikes: true,
            spike_params: SpikeParams::aggressive(),
            remove_small_components: true,
            min_component_faces: 1000,
            fill_holes: true,
            max_hole_edges: 200,
        }
    }

    /// Sets whether to remove outliers.
    #[must_use]
    pub const fn with_remove_outliers(mut self, enabled: bool) -> Self {
        self.remove_outliers = enabled;
        self
    }

    /// Sets the outlier removal parameters.
    #[must_use]
    pub const fn with_outlier_params(mut self, params: OutlierParams) -> Self {
        self.outlier_params = params;
        self
    }

    /// Sets whether to remove spikes.
    #[must_use]
    pub const fn with_remove_spikes(mut self, enabled: bool) -> Self {
        self.remove_spikes = enabled;
        self
    }

    /// Sets the spike removal parameters.
    #[must_use]
    pub const fn with_spike_params(mut self, params: SpikeParams) -> Self {
        self.spike_params = params;
        self
    }

    /// Sets whether to remove small components.
    #[must_use]
    pub const fn with_remove_small_components(mut self, enabled: bool) -> Self {
        self.remove_small_components = enabled;
        self
    }

    /// Sets the minimum component face count.
    #[must_use]
    pub const fn with_min_component_faces(mut self, min: usize) -> Self {
        self.min_component_faces = min;
        self
    }

    /// Sets whether to fill holes.
    #[must_use]
    pub const fn with_fill_holes(mut self, enabled: bool) -> Self {
        self.fill_holes = enabled;
        self
    }

    /// Sets the maximum hole size to fill.
    #[must_use]
    pub const fn with_max_hole_edges(mut self, max: usize) -> Self {
        self.max_hole_edges = max;
        self
    }
}

/// Result of the complete scan cleanup pipeline.
#[derive(Debug, Clone)]
pub struct CleanupResult {
    /// The cleaned mesh.
    pub mesh: IndexedMesh,

    /// Number of vertices in the original mesh.
    pub original_vertices: usize,

    /// Number of faces in the original mesh.
    pub original_faces: usize,

    /// Number of outlier vertices removed.
    pub outliers_removed: usize,

    /// Number of spikes handled (smoothed or removed).
    pub spikes_handled: usize,

    /// Number of small components removed.
    pub components_removed: usize,

    /// Number of holes filled.
    pub holes_filled: usize,
}

impl CleanupResult {
    /// Returns true if any cleanup was performed.
    #[must_use]
    pub const fn had_changes(&self) -> bool {
        self.outliers_removed > 0
            || self.spikes_handled > 0
            || self.components_removed > 0
            || self.holes_filled > 0
    }

    /// Returns the number of vertices removed.
    #[must_use]
    pub fn vertices_removed(&self) -> usize {
        self.original_vertices.saturating_sub(self.mesh.vertices.len())
    }

    /// Returns the number of faces removed.
    #[must_use]
    pub fn faces_removed(&self) -> usize {
        self.original_faces.saturating_sub(self.mesh.faces.len())
    }
}

impl std::fmt::Display for CleanupResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Scan cleanup: {} → {} vertices, {} → {} faces",
            self.original_vertices,
            self.mesh.vertices.len(),
            self.original_faces,
            self.mesh.faces.len()
        )?;

        if self.outliers_removed > 0 {
            write!(f, ", {} outliers removed", self.outliers_removed)?;
        }
        if self.spikes_handled > 0 {
            write!(f, ", {} spikes handled", self.spikes_handled)?;
        }
        if self.components_removed > 0 {
            write!(f, ", {} components removed", self.components_removed)?;
        }
        if self.holes_filled > 0 {
            write!(f, ", {} holes filled", self.holes_filled)?;
        }

        Ok(())
    }
}

/// Performs a complete cleanup of a scanned mesh.
///
/// This runs a pipeline of cleanup operations based on the provided parameters:
/// 1. Outlier removal
/// 2. Spike removal/smoothing
/// 3. Small component removal
/// 4. Hole filling
///
/// # Arguments
///
/// * `mesh` - The input mesh to clean
/// * `params` - Parameters controlling which operations to perform
///
/// # Returns
///
/// Detailed results including the cleaned mesh and statistics.
///
/// # Errors
///
/// Returns an error if the mesh is empty.
///
/// # Example
///
/// ```
/// use mesh_scan::cleanup::{cleanup_scan, CleanupParams};
/// use mesh_types::{IndexedMesh, Vertex};
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// let result = cleanup_scan(&mesh, &CleanupParams::default()).unwrap();
/// println!("{}", result);
/// ```
pub fn cleanup_scan(mesh: &IndexedMesh, params: &CleanupParams) -> ScanResult<CleanupResult> {
    if mesh.vertices.is_empty() {
        return Err(ScanError::EmptyMesh);
    }

    let original_vertices = mesh.vertices.len();
    let original_faces = mesh.faces.len();

    let mut current_mesh = mesh.clone();
    let mut outliers_removed = 0;
    let mut spikes_handled = 0;
    let mut components_removed = 0;
    let mut holes_filled = 0;

    // Step 1: Remove outliers
    if params.remove_outliers && !current_mesh.faces.is_empty() {
        let before_count = current_mesh.vertices.len();
        current_mesh = remove_mesh_outliers(&current_mesh, &params.outlier_params)?;
        outliers_removed = before_count.saturating_sub(current_mesh.vertices.len());
    }

    // Step 2: Remove/smooth spikes
    if params.remove_spikes && !current_mesh.faces.is_empty() {
        let spike_result = remove_spikes(&current_mesh, &params.spike_params)?;
        spikes_handled = spike_result.spikes_detected;
        current_mesh = spike_result.mesh;
    }

    // Step 3: Remove small components
    if params.remove_small_components && !current_mesh.faces.is_empty() {
        let components = find_connected_components(&current_mesh);
        let before_count = components.component_count;

        if before_count > 1 {
            // Keep only the largest component
            let _removed = keep_largest_component(&mut current_mesh);
            components_removed = before_count.saturating_sub(1);
        }
    }

    // Step 4: Fill holes
    if params.fill_holes && !current_mesh.faces.is_empty() {
        // Use mesh-repair's hole filling
        let adjacency = mesh_repair::MeshAdjacency::build(&current_mesh.faces);
        let holes = mesh_repair::detect_holes(&current_mesh, &adjacency);

        let fillable_holes: Vec<_> = holes
            .iter()
            .filter(|h| h.edge_count() <= params.max_hole_edges)
            .collect();

        for hole in fillable_holes {
            let fill_result = mesh_repair::fill_holes(&mut current_mesh, hole.edge_count());
            if fill_result.is_ok() {
                holes_filled += 1;
            }
        }
    }

    Ok(CleanupResult {
        mesh: current_mesh,
        original_vertices,
        original_faces,
        outliers_removed,
        spikes_handled,
        components_removed,
        holes_filled,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn make_simple_mesh() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    fn make_mesh_with_two_components() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        // Component 1
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        // Component 2 (disconnected)
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(11.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.5, 1.0, 0.0));
        mesh.faces.push([3, 4, 5]);

        mesh
    }

    #[test]
    fn test_cleanup_params_default() {
        let params = CleanupParams::default();
        assert!(params.remove_outliers);
        assert!(params.remove_spikes);
        assert!(params.remove_small_components);
        assert!(!params.fill_holes);
    }

    #[test]
    fn test_cleanup_params_presets() {
        let body = CleanupParams::for_body_scan();
        assert!(body.fill_holes);
        assert!(body.min_component_faces > 100);

        let object = CleanupParams::for_object_scan();
        assert!(!object.fill_holes);

        let minimal = CleanupParams::minimal();
        assert!(!minimal.remove_spikes);
        assert!(!minimal.remove_small_components);

        let aggressive = CleanupParams::aggressive();
        assert!(aggressive.fill_holes);
        assert!(aggressive.min_component_faces > 500);
    }

    #[test]
    fn test_cleanup_params_builder() {
        let params = CleanupParams::new()
            .with_remove_outliers(false)
            .with_remove_spikes(true)
            .with_remove_small_components(true)
            .with_min_component_faces(50)
            .with_fill_holes(true)
            .with_max_hole_edges(100);

        assert!(!params.remove_outliers);
        assert!(params.remove_spikes);
        assert!(params.remove_small_components);
        assert_eq!(params.min_component_faces, 50);
        assert!(params.fill_holes);
        assert_eq!(params.max_hole_edges, 100);
    }

    #[test]
    fn test_cleanup_scan_empty_mesh() {
        let mesh = IndexedMesh::new();
        let result = cleanup_scan(&mesh, &CleanupParams::default());
        assert!(matches!(result, Err(ScanError::EmptyMesh)));
    }

    #[test]
    fn test_cleanup_scan_simple_mesh() {
        let mesh = make_simple_mesh();
        let params = CleanupParams::new()
            .with_remove_outliers(false)
            .with_remove_spikes(false)
            .with_remove_small_components(false);

        let result = cleanup_scan(&mesh, &params).unwrap();

        assert_eq!(result.original_vertices, 3);
        assert_eq!(result.original_faces, 1);
        assert_eq!(result.mesh.vertices.len(), 3);
        assert_eq!(result.mesh.faces.len(), 1);
    }

    #[test]
    fn test_cleanup_scan_removes_small_components() {
        let mesh = make_mesh_with_two_components();
        let params = CleanupParams::new()
            .with_remove_outliers(false)
            .with_remove_spikes(false)
            .with_remove_small_components(true)
            .with_min_component_faces(0);

        let result = cleanup_scan(&mesh, &params).unwrap();

        // Should keep only the largest component
        assert!(result.mesh.vertices.len() <= mesh.vertices.len());
        assert!(result.components_removed > 0 || result.mesh.faces.len() == 1);
    }

    #[test]
    fn test_cleanup_result_display() {
        let result = CleanupResult {
            mesh: IndexedMesh::new(),
            original_vertices: 1000,
            original_faces: 2000,
            outliers_removed: 50,
            spikes_handled: 10,
            components_removed: 3,
            holes_filled: 2,
        };

        let display = format!("{result}");
        assert!(display.contains("1000"));
        assert!(display.contains("50"));
        assert!(display.contains("10"));
        assert!(display.contains("3"));
        assert!(display.contains("2"));
    }

    #[test]
    fn test_cleanup_result_had_changes() {
        let no_changes = CleanupResult {
            mesh: IndexedMesh::new(),
            original_vertices: 100,
            original_faces: 200,
            outliers_removed: 0,
            spikes_handled: 0,
            components_removed: 0,
            holes_filled: 0,
        };
        assert!(!no_changes.had_changes());

        let with_changes = CleanupResult {
            mesh: IndexedMesh::new(),
            original_vertices: 100,
            original_faces: 200,
            outliers_removed: 5,
            spikes_handled: 0,
            components_removed: 0,
            holes_filled: 0,
        };
        assert!(with_changes.had_changes());
    }

    #[test]
    fn test_cleanup_result_vertices_removed() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));

        let result = CleanupResult {
            mesh,
            original_vertices: 10,
            original_faces: 20,
            outliers_removed: 0,
            spikes_handled: 0,
            components_removed: 0,
            holes_filled: 0,
        };

        assert_eq!(result.vertices_removed(), 9);
    }

    #[test]
    fn test_cleanup_result_faces_removed() {
        let mut mesh = IndexedMesh::new();
        mesh.faces.push([0, 1, 2]);

        let result = CleanupResult {
            mesh,
            original_vertices: 3,
            original_faces: 5,
            outliers_removed: 0,
            spikes_handled: 0,
            components_removed: 0,
            holes_filled: 0,
        };

        assert_eq!(result.faces_removed(), 4);
    }
}
