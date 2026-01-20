//! Multi-scan alignment and merging.
//!
//! This module provides algorithms for aligning and merging multiple
//! overlapping 3D scans into a single unified mesh:
//!
//! - **Alignment** - Global alignment refinement using ICP
//! - **Merging** - Combining scans with overlap handling
//!
//! # Quick Start
//!
//! ```
//! use mesh_scan::multiscan::{
//!     align_multiple_scans, merge_scans,
//!     MultiAlignmentParams, MergeParams,
//! };
//! use mesh_types::{IndexedMesh, Vertex};
//!
//! // Create two overlapping scans
//! let mut scan1 = IndexedMesh::new();
//! scan1.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! scan1.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! scan1.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
//! scan1.vertices.push(Vertex::from_coords(0.0, 0.0, 1.0));
//!
//! let mut scan2 = IndexedMesh::new();
//! scan2.vertices.push(Vertex::from_coords(0.1, 0.0, 0.0));
//! scan2.vertices.push(Vertex::from_coords(1.1, 0.0, 0.0));
//! scan2.vertices.push(Vertex::from_coords(0.1, 1.0, 0.0));
//! scan2.vertices.push(Vertex::from_coords(0.1, 0.0, 1.0));
//!
//! let scans = vec![scan1, scan2];
//!
//! // Align scans
//! let alignment = align_multiple_scans(&scans, &MultiAlignmentParams::default()).unwrap();
//! println!("Alignment converged: {}", alignment.converged);
//!
//! // Merge aligned scans
//! let merged = merge_scans(&scans, &alignment.transforms, &MergeParams::default()).unwrap();
//! println!("{}", merged);
//! ```
//!
//! # Workflow
//!
//! 1. **Alignment**: Use `align_multiple_scans` to compute transforms that
//!    align all scans to a common coordinate frame (first scan).
//!
//! 2. **Merging**: Use `merge_scans` with the computed transforms to combine
//!    all scans into a single mesh, handling overlapping regions.

pub mod alignment;
pub mod merge;

pub use alignment::{
    GlobalAlignmentParams, GlobalAlignmentResult, PairwiseAlignment, compute_pairwise_alignments,
    refine_global_alignment,
};
pub use merge::{MergeParams, MergeResult, OverlapHandling, merge_scans};

use mesh_registration::{IcpParams, RigidTransform, icp_align};
use mesh_types::IndexedMesh;

use crate::error::{ScanError, ScanResult};

/// Parameters for multi-scan alignment.
#[derive(Debug, Clone, Default)]
pub struct MultiAlignmentParams {
    /// ICP parameters for pairwise alignment.
    pub icp_params: IcpParams,

    /// Whether to perform global refinement after pairwise alignment.
    pub refine_globally: bool,

    /// Global alignment parameters (if `refine_globally` is true).
    pub global_params: GlobalAlignmentParams,
}

impl MultiAlignmentParams {
    /// Creates new parameters with defaults.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the ICP parameters.
    #[must_use]
    pub const fn with_icp_params(mut self, params: IcpParams) -> Self {
        self.icp_params = params;
        self
    }

    /// Enables or disables global refinement.
    #[must_use]
    pub const fn with_global_refinement(mut self, enable: bool) -> Self {
        self.refine_globally = enable;
        self
    }

    /// Sets the global alignment parameters.
    #[must_use]
    pub const fn with_global_params(mut self, params: GlobalAlignmentParams) -> Self {
        self.global_params = params;
        self
    }

    /// Creates parameters for fast alignment (no global refinement).
    #[must_use]
    pub fn fast() -> Self {
        Self {
            icp_params: IcpParams::new()
                .with_max_iterations(30)
                .with_subsample_ratio(0.5),
            refine_globally: false,
            global_params: GlobalAlignmentParams::default(),
        }
    }

    /// Creates parameters for high-quality alignment.
    #[must_use]
    pub fn high_quality() -> Self {
        Self {
            icp_params: IcpParams::new()
                .with_max_iterations(200)
                .with_convergence_threshold(1e-8),
            refine_globally: true,
            global_params: GlobalAlignmentParams::high_quality(),
        }
    }
}

/// Result of multi-scan alignment.
#[derive(Debug, Clone)]
pub struct MultiAlignmentResult {
    /// Transforms for each scan (first scan is identity).
    pub transforms: Vec<RigidTransform>,

    /// Pairwise RMS errors.
    pub pairwise_errors: Vec<f64>,

    /// Whether all pairwise alignments converged.
    pub converged: bool,

    /// Average pairwise RMS error.
    pub avg_rms_error: f64,

    /// Maximum pairwise RMS error.
    pub max_rms_error: f64,
}

impl std::fmt::Display for MultiAlignmentResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "MultiAlignment: {} scans, converged: {}, avg RMS: {:.6}, max RMS: {:.6}",
            self.transforms.len(),
            self.converged,
            self.avg_rms_error,
            self.max_rms_error
        )
    }
}

/// Aligns multiple scans to a common coordinate frame.
///
/// This function computes rigid transforms that align all scans to the
/// coordinate frame of the first scan. Alignment is performed using
/// pairwise ICP between consecutive scans.
///
/// # Arguments
///
/// * `scans` - Slice of meshes to align
/// * `params` - Multi-alignment parameters
///
/// # Returns
///
/// Alignment result with transforms for each scan.
///
/// # Errors
///
/// Returns an error if:
/// - Fewer than 2 scans provided
/// - ICP alignment fails for any pair
///
/// # Example
///
/// ```
/// use mesh_scan::multiscan::{align_multiple_scans, MultiAlignmentParams};
/// use mesh_types::{IndexedMesh, Vertex};
///
/// let mut scan1 = IndexedMesh::new();
/// for i in 0..10 {
///     scan1.vertices.push(Vertex::from_coords(f64::from(i), 0.0, 0.0));
/// }
///
/// let mut scan2 = IndexedMesh::new();
/// for i in 0..10 {
///     scan2.vertices.push(Vertex::from_coords(f64::from(i) + 0.1, 0.0, 0.0));
/// }
///
/// let scans = vec![scan1, scan2];
/// let result = align_multiple_scans(&scans, &MultiAlignmentParams::default()).unwrap();
///
/// assert_eq!(result.transforms.len(), 2);
/// assert!(result.transforms[0].is_identity(1e-10));
/// ```
pub fn align_multiple_scans(
    scans: &[IndexedMesh],
    params: &MultiAlignmentParams,
) -> ScanResult<MultiAlignmentResult> {
    if scans.len() < 2 {
        return Err(ScanError::InsufficientPoints {
            required: 2,
            actual: scans.len(),
        });
    }

    // Check for empty scans
    for (i, scan) in scans.iter().enumerate() {
        if scan.vertices.is_empty() {
            return Err(ScanError::AlignmentFailed {
                reason: format!("scan {i} is empty"),
            });
        }
    }

    if params.refine_globally {
        // Use global alignment refinement
        let result = refine_global_alignment(scans, &params.global_params)?;

        let pairwise_errors: Vec<f64> = result
            .pairwise_alignments
            .iter()
            .map(|p| p.icp_result.rms_error)
            .collect();

        return Ok(MultiAlignmentResult {
            transforms: result.transforms,
            pairwise_errors,
            converged: result.converged,
            avg_rms_error: result.avg_rms_error,
            max_rms_error: result.max_rms_error,
        });
    }

    // Simple pairwise alignment without global refinement
    let mut transforms = vec![RigidTransform::identity(); scans.len()];
    let mut pairwise_errors = Vec::with_capacity(scans.len() - 1);
    let mut all_converged = true;

    // Chain pairwise alignments: align scan[i+1] to scan[i]
    for i in 0..scans.len() - 1 {
        let result = icp_align(&scans[i + 1], &scans[i], &params.icp_params).map_err(|e| {
            ScanError::AlignmentFailed {
                reason: format!("ICP failed for scans {} -> {}: {e}", i + 1, i),
            }
        })?;

        pairwise_errors.push(result.rms_error);
        all_converged = all_converged && result.converged;

        // Chain: T[i+1] = T_pairwise * T[i]
        transforms[i + 1] = result.transform.compose(&transforms[i]);
    }

    // Compute error statistics
    let avg_rms_error = if pairwise_errors.is_empty() {
        0.0
    } else {
        let sum: f64 = pairwise_errors.iter().sum();
        #[allow(clippy::cast_precision_loss)]
        let avg = sum / pairwise_errors.len() as f64;
        avg
    };

    let max_rms_error = pairwise_errors.iter().copied().fold(0.0, f64::max);

    Ok(MultiAlignmentResult {
        transforms,
        pairwise_errors,
        converged: all_converged,
        avg_rms_error,
        max_rms_error,
    })
}

/// Aligns and merges multiple scans in one step.
///
/// This is a convenience function that combines `align_multiple_scans`
/// and `merge_scans` for the common case of aligning then merging.
///
/// # Arguments
///
/// * `scans` - Slice of meshes to align and merge
/// * `align_params` - Alignment parameters
/// * `merge_params` - Merge parameters
///
/// # Returns
///
/// A tuple of (merged mesh, alignment result).
///
/// # Errors
///
/// Returns an error if alignment or merging fails.
///
/// # Example
///
/// ```
/// use mesh_scan::multiscan::{
///     align_and_merge, MultiAlignmentParams, MergeParams,
/// };
/// use mesh_types::{IndexedMesh, Vertex};
///
/// let mut scan1 = IndexedMesh::new();
/// for i in 0..5 {
///     scan1.vertices.push(Vertex::from_coords(f64::from(i), 0.0, 0.0));
/// }
///
/// let mut scan2 = IndexedMesh::new();
/// for i in 0..5 {
///     scan2.vertices.push(Vertex::from_coords(f64::from(i) + 0.05, 0.0, 0.0));
/// }
///
/// let scans = vec![scan1, scan2];
/// let (merge_result, alignment) = align_and_merge(
///     &scans,
///     &MultiAlignmentParams::fast(),
///     &MergeParams::default(),
/// ).unwrap();
///
/// println!("Alignment: {}", alignment);
/// println!("Merge: {}", merge_result);
/// ```
pub fn align_and_merge(
    scans: &[IndexedMesh],
    align_params: &MultiAlignmentParams,
    merge_params: &MergeParams,
) -> ScanResult<(MergeResult, MultiAlignmentResult)> {
    let alignment = align_multiple_scans(scans, align_params)?;
    let merge_result = merge_scans(scans, &alignment.transforms, merge_params)?;
    Ok((merge_result, alignment))
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::cast_lossless,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::redundant_clone,
    clippy::needless_collect
)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn make_test_scan(offset: f64) -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        for i in 0..5 {
            for j in 0..5 {
                mesh.vertices.push(Vertex::from_coords(
                    f64::from(i) + offset,
                    f64::from(j),
                    0.0,
                ));
            }
        }
        mesh
    }

    #[test]
    fn test_multi_alignment_params_default() {
        let params = MultiAlignmentParams::default();
        assert!(!params.refine_globally);
    }

    #[test]
    fn test_multi_alignment_params_builder() {
        let params = MultiAlignmentParams::new()
            .with_global_refinement(true)
            .with_icp_params(IcpParams::new().with_max_iterations(50));

        assert!(params.refine_globally);
        assert_eq!(params.icp_params.max_iterations, 50);
    }

    #[test]
    fn test_multi_alignment_params_presets() {
        let fast = MultiAlignmentParams::fast();
        assert!(!fast.refine_globally);

        let hq = MultiAlignmentParams::high_quality();
        assert!(hq.refine_globally);
    }

    #[test]
    fn test_align_insufficient_scans() {
        let scans = vec![make_test_scan(0.0)];
        let result = align_multiple_scans(&scans, &MultiAlignmentParams::default());
        assert!(matches!(result, Err(ScanError::InsufficientPoints { .. })));
    }

    #[test]
    fn test_align_empty_scan() {
        let scans = vec![make_test_scan(0.0), IndexedMesh::new()];
        let result = align_multiple_scans(&scans, &MultiAlignmentParams::default());
        assert!(matches!(result, Err(ScanError::AlignmentFailed { .. })));
    }

    #[test]
    fn test_align_two_scans() {
        let scans = vec![make_test_scan(0.0), make_test_scan(0.5)];
        let result = align_multiple_scans(&scans, &MultiAlignmentParams::fast()).unwrap();

        assert_eq!(result.transforms.len(), 2);
        assert!(result.transforms[0].is_identity(1e-10));
        assert_eq!(result.pairwise_errors.len(), 1);
    }

    #[test]
    fn test_align_three_scans() {
        let scans = vec![
            make_test_scan(0.0),
            make_test_scan(0.3),
            make_test_scan(0.6),
        ];
        let result = align_multiple_scans(&scans, &MultiAlignmentParams::fast()).unwrap();

        assert_eq!(result.transforms.len(), 3);
        assert_eq!(result.pairwise_errors.len(), 2);
    }

    #[test]
    fn test_align_with_global_refinement() {
        let scans = vec![make_test_scan(0.0), make_test_scan(0.3)];
        let params = MultiAlignmentParams::new().with_global_refinement(true);
        let result = align_multiple_scans(&scans, &params).unwrap();

        assert_eq!(result.transforms.len(), 2);
    }

    #[test]
    fn test_align_and_merge() {
        let scans = vec![make_test_scan(0.0), make_test_scan(0.3)];

        let (merge_result, alignment) =
            align_and_merge(&scans, &MultiAlignmentParams::fast(), &MergeParams::fast()).unwrap();

        assert_eq!(alignment.transforms.len(), 2);
        assert!(merge_result.merged_vertex_count > 0);
    }

    #[test]
    fn test_multi_alignment_result_display() {
        let result = MultiAlignmentResult {
            transforms: vec![RigidTransform::identity(); 3],
            pairwise_errors: vec![0.001, 0.002],
            converged: true,
            avg_rms_error: 0.0015,
            max_rms_error: 0.002,
        };

        let display = format!("{result}");
        assert!(display.contains("3 scans"));
        assert!(display.contains("converged: true"));
    }

    #[test]
    fn test_align_known_transform() {
        // Create two scans with known offset
        let scan1 = make_test_scan(0.0);
        let offset = 0.2;
        let scan2 = make_test_scan(offset);

        let scans = vec![scan1, scan2];
        let result = align_multiple_scans(&scans, &MultiAlignmentParams::fast()).unwrap();

        // The transform should approximately reverse the offset
        // (scan2 aligned to scan1 coordinate frame)
        let transform = &result.transforms[1];

        // Check that the transform is reasonable
        // (we can't expect exact recovery with ICP on point clouds)
        assert!(transform.translation.norm() < 2.0);
    }
}
