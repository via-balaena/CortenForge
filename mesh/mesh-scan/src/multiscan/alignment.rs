//! Global alignment refinement for multiple scans.
//!
//! This module provides algorithms for refining the alignment of multiple
//! overlapping scans into a consistent global coordinate frame.

use mesh_registration::{IcpParams, IcpResult, RigidTransform, icp_align};
use mesh_types::IndexedMesh;
use std::collections::HashMap;

use crate::error::{ScanError, ScanResult};

/// Parameters for global alignment refinement.
#[derive(Debug, Clone)]
pub struct GlobalAlignmentParams {
    /// ICP parameters for pairwise alignment.
    pub icp_params: IcpParams,

    /// Maximum number of global refinement iterations.
    pub max_global_iterations: u32,

    /// Convergence threshold for global alignment (RMS change).
    pub global_convergence_threshold: f64,

    /// Minimum overlap ratio required for a valid pairwise alignment.
    pub min_overlap_ratio: f64,

    /// Whether to use loop closure constraints.
    pub use_loop_closure: bool,
}

impl Default for GlobalAlignmentParams {
    fn default() -> Self {
        Self {
            icp_params: IcpParams::default(),
            max_global_iterations: 10,
            global_convergence_threshold: 1e-4,
            min_overlap_ratio: 0.1,
            use_loop_closure: true,
        }
    }
}

impl GlobalAlignmentParams {
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

    /// Sets the maximum number of global iterations.
    #[must_use]
    pub const fn with_max_global_iterations(mut self, iterations: u32) -> Self {
        self.max_global_iterations = iterations;
        self
    }

    /// Sets the global convergence threshold.
    #[must_use]
    pub const fn with_global_convergence_threshold(mut self, threshold: f64) -> Self {
        self.global_convergence_threshold = threshold;
        self
    }

    /// Sets the minimum overlap ratio.
    #[must_use]
    pub const fn with_min_overlap_ratio(mut self, ratio: f64) -> Self {
        self.min_overlap_ratio = ratio;
        self
    }

    /// Enables or disables loop closure.
    #[must_use]
    pub const fn with_loop_closure(mut self, enable: bool) -> Self {
        self.use_loop_closure = enable;
        self
    }

    /// Creates parameters for high-quality alignment.
    #[must_use]
    pub fn high_quality() -> Self {
        Self {
            icp_params: IcpParams::new()
                .with_max_iterations(200)
                .with_convergence_threshold(1e-8),
            max_global_iterations: 20,
            global_convergence_threshold: 1e-6,
            min_overlap_ratio: 0.15,
            use_loop_closure: true,
        }
    }

    /// Creates parameters for fast alignment.
    #[must_use]
    pub fn fast() -> Self {
        Self {
            icp_params: IcpParams::new()
                .with_max_iterations(30)
                .with_subsample_ratio(0.5),
            max_global_iterations: 3,
            global_convergence_threshold: 1e-3,
            min_overlap_ratio: 0.05,
            use_loop_closure: false,
        }
    }
}

/// Result of pairwise alignment between two scans.
#[derive(Debug, Clone)]
pub struct PairwiseAlignment {
    /// Index of the source scan.
    pub source_idx: usize,

    /// Index of the target scan.
    pub target_idx: usize,

    /// Transform from source to target.
    pub transform: RigidTransform,

    /// ICP result with error metrics.
    pub icp_result: IcpResult,

    /// Estimated overlap ratio.
    pub overlap_ratio: f64,
}

/// Result of global alignment.
#[derive(Debug, Clone)]
pub struct GlobalAlignmentResult {
    /// Transforms for each scan (first scan is identity).
    pub transforms: Vec<RigidTransform>,

    /// Pairwise alignment results.
    pub pairwise_alignments: Vec<PairwiseAlignment>,

    /// Number of global iterations performed.
    pub iterations: u32,

    /// Whether the alignment converged.
    pub converged: bool,

    /// Final average pairwise RMS error.
    pub avg_rms_error: f64,

    /// Final maximum pairwise RMS error.
    pub max_rms_error: f64,
}

impl std::fmt::Display for GlobalAlignmentResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "GlobalAlignment: {} scans, {} iterations, converged: {}, avg RMS: {:.6}",
            self.transforms.len(),
            self.iterations,
            self.converged,
            self.avg_rms_error
        )
    }
}

/// Computes pairwise ICP alignment between adjacent scans.
///
/// # Arguments
///
/// * `scans` - Slice of meshes to align
/// * `params` - Global alignment parameters
///
/// # Returns
///
/// Vector of pairwise alignment results for adjacent pairs.
///
/// # Errors
///
/// Returns an error if ICP alignment fails for any pair.
pub fn compute_pairwise_alignments(
    scans: &[IndexedMesh],
    params: &GlobalAlignmentParams,
) -> ScanResult<Vec<PairwiseAlignment>> {
    if scans.len() < 2 {
        return Ok(Vec::new());
    }

    let mut alignments = Vec::with_capacity(scans.len() - 1);

    // Align consecutive pairs
    for i in 0..scans.len() - 1 {
        let result = icp_align(&scans[i + 1], &scans[i], &params.icp_params).map_err(|e| {
            ScanError::AlignmentFailed {
                reason: format!("ICP failed for scans {} -> {}: {e}", i + 1, i),
            }
        })?;

        // Estimate overlap ratio from correspondence count
        #[allow(clippy::cast_precision_loss)]
        let overlap_ratio =
            result.correspondence_count as f64 / scans[i + 1].vertices.len().max(1) as f64;

        alignments.push(PairwiseAlignment {
            source_idx: i + 1,
            target_idx: i,
            transform: result.transform,
            icp_result: result,
            overlap_ratio,
        });
    }

    // Add loop closure if enabled and we have enough scans
    if params.use_loop_closure && scans.len() >= 3 {
        // Try to align last scan to first
        let result = icp_align(&scans[scans.len() - 1], &scans[0], &params.icp_params);

        if let Ok(result) = result {
            #[allow(clippy::cast_precision_loss)]
            let overlap_ratio = result.correspondence_count as f64
                / scans[scans.len() - 1].vertices.len().max(1) as f64;

            if overlap_ratio >= params.min_overlap_ratio {
                alignments.push(PairwiseAlignment {
                    source_idx: scans.len() - 1,
                    target_idx: 0,
                    transform: result.transform,
                    icp_result: result,
                    overlap_ratio,
                });
            }
        }
    }

    Ok(alignments)
}

/// Chains pairwise transforms to compute global transforms.
///
/// Each scan's global transform is computed by composing all transforms
/// from scan 0 to that scan.
fn chain_transforms(pairwise: &[PairwiseAlignment], num_scans: usize) -> Vec<RigidTransform> {
    let mut global = vec![RigidTransform::identity(); num_scans];

    // Build adjacency map for transform lookup
    let mut transform_map: HashMap<(usize, usize), &RigidTransform> = HashMap::new();
    for alignment in pairwise {
        transform_map.insert(
            (alignment.source_idx, alignment.target_idx),
            &alignment.transform,
        );
    }

    // Chain transforms from scan 0
    for i in 1..num_scans {
        // Compose transforms from scan 0 to scan i
        let mut composed = RigidTransform::identity();

        for j in (1..=i).rev() {
            if let Some(t) = transform_map.get(&(j, j - 1)) {
                composed = composed.compose(t);
            }
        }

        global[i] = composed;
    }

    global
}

/// Performs global alignment refinement on multiple scans.
///
/// This function aligns multiple overlapping scans into a consistent
/// global coordinate frame using iterative pairwise ICP refinement.
///
/// # Arguments
///
/// * `scans` - Slice of meshes to align
/// * `params` - Global alignment parameters
///
/// # Returns
///
/// Global alignment result with transforms for each scan.
///
/// # Errors
///
/// Returns an error if:
/// - Fewer than 2 scans provided
/// - ICP alignment fails
///
/// # Example
///
/// ```
/// use mesh_scan::multiscan::{refine_global_alignment, GlobalAlignmentParams};
/// use mesh_types::{IndexedMesh, Vertex};
///
/// let mut scan1 = IndexedMesh::new();
/// scan1.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// scan1.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// scan1.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
/// scan1.vertices.push(Vertex::from_coords(0.0, 0.0, 1.0));
///
/// let mut scan2 = IndexedMesh::new();
/// scan2.vertices.push(Vertex::from_coords(0.1, 0.0, 0.0));
/// scan2.vertices.push(Vertex::from_coords(1.1, 0.0, 0.0));
/// scan2.vertices.push(Vertex::from_coords(0.1, 1.0, 0.0));
/// scan2.vertices.push(Vertex::from_coords(0.1, 0.0, 1.0));
///
/// let scans = vec![scan1, scan2];
/// let result = refine_global_alignment(&scans, &GlobalAlignmentParams::default()).unwrap();
/// println!("Aligned {} scans", result.transforms.len());
/// ```
pub fn refine_global_alignment(
    scans: &[IndexedMesh],
    params: &GlobalAlignmentParams,
) -> ScanResult<GlobalAlignmentResult> {
    if scans.len() < 2 {
        return Err(ScanError::InsufficientPoints {
            required: 2,
            actual: scans.len(),
        });
    }

    // Initial pairwise alignment
    let mut pairwise = compute_pairwise_alignments(scans, params)?;

    // Chain to get initial global transforms
    let mut global_transforms = chain_transforms(&pairwise, scans.len());

    // Iteratively refine
    let mut converged = false;
    let mut iterations = 0;
    let mut prev_avg_error = f64::MAX;

    for iter in 0..params.max_global_iterations {
        iterations = iter + 1;

        // Transform all scans to global frame
        let transformed_scans: Vec<IndexedMesh> = scans
            .iter()
            .zip(global_transforms.iter())
            .map(|(scan, transform)| transform_mesh(scan, transform))
            .collect();

        // Re-compute pairwise alignments in global frame
        let new_pairwise = compute_pairwise_alignments(&transformed_scans, params)?;

        // Update global transforms by composing with refinements
        for alignment in &new_pairwise {
            let idx = alignment.source_idx;
            if idx < global_transforms.len() {
                global_transforms[idx] = alignment.transform.compose(&global_transforms[idx]);
            }
        }

        // Compute average error
        let avg_error = compute_average_error(&new_pairwise);

        // Check convergence
        let error_change = (prev_avg_error - avg_error).abs();
        if error_change < params.global_convergence_threshold {
            converged = true;
            pairwise = new_pairwise;
            break;
        }

        prev_avg_error = avg_error;
        pairwise = new_pairwise;
    }

    // Compute final error metrics
    let (avg_rms, max_rms) = compute_error_summary(&pairwise);

    Ok(GlobalAlignmentResult {
        transforms: global_transforms,
        pairwise_alignments: pairwise,
        iterations,
        converged,
        avg_rms_error: avg_rms,
        max_rms_error: max_rms,
    })
}

/// Transforms a mesh by a rigid transform.
fn transform_mesh(mesh: &IndexedMesh, transform: &RigidTransform) -> IndexedMesh {
    let mut result = mesh.clone();
    for v in &mut result.vertices {
        v.position = transform.transform_point(&v.position);
        if let Some(normal) = v.attributes.normal {
            v.attributes.normal = Some(transform.rotation * normal);
        }
    }
    result
}

/// Computes average RMS error from pairwise alignments.
fn compute_average_error(pairwise: &[PairwiseAlignment]) -> f64 {
    if pairwise.is_empty() {
        return 0.0;
    }

    let sum: f64 = pairwise.iter().map(|p| p.icp_result.rms_error).sum();

    #[allow(clippy::cast_precision_loss)]
    let avg = sum / pairwise.len() as f64;
    avg
}

/// Computes average and max RMS error from pairwise alignments.
fn compute_error_summary(pairwise: &[PairwiseAlignment]) -> (f64, f64) {
    if pairwise.is_empty() {
        return (0.0, 0.0);
    }

    let sum: f64 = pairwise.iter().map(|p| p.icp_result.rms_error).sum();
    let max = pairwise
        .iter()
        .map(|p| p.icp_result.rms_error)
        .fold(0.0, f64::max);

    #[allow(clippy::cast_precision_loss)]
    let avg = sum / pairwise.len() as f64;
    (avg, max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;
    use nalgebra::Vector3;

    fn make_test_mesh(offset: f64) -> IndexedMesh {
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
    fn test_global_alignment_params_default() {
        let params = GlobalAlignmentParams::default();
        assert_eq!(params.max_global_iterations, 10);
        assert!(params.use_loop_closure);
    }

    #[test]
    fn test_global_alignment_params_builder() {
        let params = GlobalAlignmentParams::new()
            .with_max_global_iterations(5)
            .with_loop_closure(false)
            .with_min_overlap_ratio(0.2);

        assert_eq!(params.max_global_iterations, 5);
        assert!(!params.use_loop_closure);
        assert!((params.min_overlap_ratio - 0.2).abs() < 1e-10);
    }

    #[test]
    fn test_global_alignment_params_presets() {
        let high = GlobalAlignmentParams::high_quality();
        assert_eq!(high.max_global_iterations, 20);

        let fast = GlobalAlignmentParams::fast();
        assert_eq!(fast.max_global_iterations, 3);
    }

    #[test]
    fn test_pairwise_alignment_single_scan() {
        let scans = vec![make_test_mesh(0.0)];
        let params = GlobalAlignmentParams::default();
        let result = compute_pairwise_alignments(&scans, &params).unwrap();
        assert!(result.is_empty());
    }

    #[test]
    fn test_pairwise_alignment_two_scans() {
        let scans = vec![make_test_mesh(0.0), make_test_mesh(0.5)];
        let params = GlobalAlignmentParams::fast();
        let result = compute_pairwise_alignments(&scans, &params).unwrap();
        assert!(!result.is_empty());
        assert_eq!(result[0].source_idx, 1);
        assert_eq!(result[0].target_idx, 0);
    }

    #[test]
    fn test_refine_global_alignment_insufficient_scans() {
        let scans = vec![make_test_mesh(0.0)];
        let params = GlobalAlignmentParams::default();
        let result = refine_global_alignment(&scans, &params);
        assert!(matches!(result, Err(ScanError::InsufficientPoints { .. })));
    }

    #[test]
    fn test_refine_global_alignment_two_scans() {
        let scans = vec![make_test_mesh(0.0), make_test_mesh(0.3)];
        let params = GlobalAlignmentParams::fast();
        let result = refine_global_alignment(&scans, &params).unwrap();

        assert_eq!(result.transforms.len(), 2);
        // First transform should be identity
        assert!(result.transforms[0].is_identity(1e-10));
    }

    #[test]
    fn test_chain_transforms() {
        let alignments = vec![
            PairwiseAlignment {
                source_idx: 1,
                target_idx: 0,
                transform: RigidTransform::from_translation(Vector3::new(1.0, 0.0, 0.0)),
                icp_result: IcpResult {
                    transform: RigidTransform::identity(),
                    rms_error: 0.0,
                    max_error: 0.0,
                    iterations: 1,
                    converged: true,
                    correspondence_count: 10,
                },
                overlap_ratio: 1.0,
            },
            PairwiseAlignment {
                source_idx: 2,
                target_idx: 1,
                transform: RigidTransform::from_translation(Vector3::new(2.0, 0.0, 0.0)),
                icp_result: IcpResult {
                    transform: RigidTransform::identity(),
                    rms_error: 0.0,
                    max_error: 0.0,
                    iterations: 1,
                    converged: true,
                    correspondence_count: 10,
                },
                overlap_ratio: 1.0,
            },
        ];

        let global = chain_transforms(&alignments, 3);

        assert_eq!(global.len(), 3);
        assert!(global[0].is_identity(1e-10));
        // Scan 1 should be translated by (1, 0, 0)
        assert!((global[1].translation.x - 1.0).abs() < 1e-10);
        // Scan 2 should be translated by (3, 0, 0) = (1,0,0) + (2,0,0)
        assert!((global[2].translation.x - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_global_alignment_result_display() {
        let result = GlobalAlignmentResult {
            transforms: vec![RigidTransform::identity(); 3],
            pairwise_alignments: Vec::new(),
            iterations: 5,
            converged: true,
            avg_rms_error: 0.001,
            max_rms_error: 0.002,
        };

        let display = format!("{result}");
        assert!(display.contains("3 scans"));
        assert!(display.contains("5 iterations"));
        assert!(display.contains("converged: true"));
    }
}
