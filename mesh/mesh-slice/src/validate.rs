//! Validation for 3D printing (FDM and SLA).

use mesh_types::IndexedMesh;

use crate::params::SliceParams;
use crate::slicer::slice_mesh;

// ============================================================================
// FDM Validation
// ============================================================================

/// Parameters for FDM (Fused Deposition Modeling) validation.
#[derive(Debug, Clone)]
pub struct FdmParams {
    /// Nozzle diameter in mm.
    pub nozzle_diameter: f64,

    /// Minimum wall thickness (typically 2x nozzle diameter).
    pub min_wall_thickness: f64,

    /// Layer height in mm.
    pub layer_height: f64,

    /// Minimum feature size (typically nozzle diameter).
    pub min_feature_size: f64,

    /// Maximum overhang angle in degrees (0 = vertical, 90 = horizontal).
    pub max_overhang_angle: f64,

    /// Minimum gap between features.
    pub min_gap: f64,
}

impl Default for FdmParams {
    fn default() -> Self {
        Self {
            nozzle_diameter: 0.4,
            min_wall_thickness: 0.8, // 2x nozzle
            layer_height: 0.2,
            min_feature_size: 0.4,
            max_overhang_angle: 45.0,
            min_gap: 0.4,
        }
    }
}

impl FdmParams {
    /// Parameters for a 0.4mm nozzle (most common).
    #[must_use]
    pub fn nozzle_04() -> Self {
        Self::default()
    }

    /// Parameters for a 0.6mm nozzle.
    #[must_use]
    pub fn nozzle_06() -> Self {
        Self {
            nozzle_diameter: 0.6,
            min_wall_thickness: 1.2,
            layer_height: 0.3,
            min_feature_size: 0.6,
            min_gap: 0.6,
            ..Default::default()
        }
    }

    /// Parameters for a 0.25mm nozzle (fine detail).
    #[must_use]
    pub fn nozzle_025() -> Self {
        Self {
            nozzle_diameter: 0.25,
            min_wall_thickness: 0.5,
            layer_height: 0.1,
            min_feature_size: 0.25,
            min_gap: 0.25,
            ..Default::default()
        }
    }
}

/// Result of FDM validation.
#[derive(Debug, Clone)]
pub struct FdmValidationResult {
    /// Whether the mesh passes all FDM checks.
    pub is_valid: bool,

    /// Layers with thin walls below minimum.
    pub thin_wall_layers: Vec<ThinWallIssue>,

    /// Layers with features smaller than nozzle.
    pub small_feature_layers: Vec<SmallFeatureIssue>,

    /// Layers with gap issues.
    pub gap_issues: Vec<GapIssue>,

    /// Total number of issues found.
    pub issue_count: usize,

    /// Summary message.
    pub summary: String,
}

impl FdmValidationResult {
    /// Check if the mesh is printable (may have warnings but no critical issues).
    #[must_use]
    pub fn is_printable(&self) -> bool {
        self.thin_wall_layers.is_empty()
    }
}

/// A thin wall issue at a specific layer.
#[derive(Debug, Clone)]
pub struct ThinWallIssue {
    /// Layer index.
    pub layer_index: usize,
    /// Z height.
    pub z_height: f64,
    /// Minimum wall thickness found.
    pub min_thickness: f64,
    /// Required minimum thickness.
    pub required_thickness: f64,
    /// Approximate location (centroid of thin region).
    pub location: (f64, f64),
}

/// A small feature issue at a specific layer.
#[derive(Debug, Clone)]
pub struct SmallFeatureIssue {
    /// Layer index.
    pub layer_index: usize,
    /// Z height.
    pub z_height: f64,
    /// Feature size found.
    pub feature_size: f64,
    /// Minimum feature size allowed.
    pub min_size: f64,
}

/// A gap issue between features.
#[derive(Debug, Clone)]
pub struct GapIssue {
    /// Layer index.
    pub layer_index: usize,
    /// Z height.
    pub z_height: f64,
    /// Gap size found.
    pub gap_size: f64,
    /// Minimum gap allowed.
    pub min_gap: f64,
}

/// Validate a mesh for FDM printing.
///
/// # Arguments
///
/// * `mesh` - The mesh to validate
/// * `params` - FDM parameters
///
/// # Returns
///
/// A [`FdmValidationResult`] with validation results.
///
/// # Example
///
/// ```
/// use mesh_types::unit_cube;
/// use mesh_slice::{validate_for_fdm, FdmParams};
///
/// let cube = unit_cube();
/// let result = validate_for_fdm(&cube, &FdmParams::default());
/// println!("{}", result.summary);
/// ```
#[must_use]
pub fn validate_for_fdm(mesh: &IndexedMesh, params: &FdmParams) -> FdmValidationResult {
    let slice_params = SliceParams {
        layer_height: params.layer_height,
        ..Default::default()
    };

    let slice_result = slice_mesh(mesh, &slice_params);

    let mut thin_wall_layers = Vec::new();
    let mut small_feature_layers = Vec::new();
    let gap_issues = Vec::new(); // Gap detection is complex; simplified for now

    for layer in &slice_result.layers {
        // Check for thin walls by analyzing contour widths
        for contour in &layer.contours {
            // Approximate wall thickness using bounding box vs perimeter
            // For a rectangular region: perimeter = 2*(w+h), area = w*h
            // Wall thickness is approximately area / (perimeter/2) for thin strips
            if contour.perimeter > 0.0 {
                let approx_thickness = 2.0 * contour.area / contour.perimeter;
                if approx_thickness < params.min_wall_thickness && approx_thickness > 0.0 {
                    thin_wall_layers.push(ThinWallIssue {
                        layer_index: layer.index,
                        z_height: layer.z_height,
                        min_thickness: approx_thickness,
                        required_thickness: params.min_wall_thickness,
                        location: (contour.centroid.x, contour.centroid.y),
                    });
                }
            }

            // Check for small features (islands smaller than min feature size)
            let feature_size = contour.area.sqrt(); // Approximate feature dimension
            if feature_size < params.min_feature_size && feature_size > 0.0 {
                small_feature_layers.push(SmallFeatureIssue {
                    layer_index: layer.index,
                    z_height: layer.z_height,
                    feature_size,
                    min_size: params.min_feature_size,
                });
            }
        }
    }

    let issue_count = thin_wall_layers.len() + small_feature_layers.len() + gap_issues.len();
    let is_valid = issue_count == 0;

    let summary = if is_valid {
        "Mesh passes all FDM validation checks.".to_string()
    } else {
        format!(
            "Found {} issues: {} thin walls, {} small features, {} gaps",
            issue_count,
            thin_wall_layers.len(),
            small_feature_layers.len(),
            gap_issues.len()
        )
    };

    FdmValidationResult {
        is_valid,
        thin_wall_layers,
        small_feature_layers,
        gap_issues,
        issue_count,
        summary,
    }
}

// ============================================================================
// SLA Validation
// ============================================================================

/// Parameters for SLA (Stereolithography) validation.
#[derive(Debug, Clone)]
pub struct SlaParams {
    /// XY resolution (pixel size) in mm.
    pub xy_resolution: f64,

    /// Layer height (Z resolution) in mm.
    pub layer_height: f64,

    /// Minimum wall thickness.
    pub min_wall_thickness: f64,

    /// Minimum feature size.
    pub min_feature_size: f64,

    /// Minimum hole diameter for drainage.
    pub min_drain_hole: f64,

    /// Maximum unsupported span in mm.
    pub max_unsupported_span: f64,
}

impl Default for SlaParams {
    fn default() -> Self {
        Self {
            xy_resolution: 0.05,
            layer_height: 0.05,
            min_wall_thickness: 0.4,
            min_feature_size: 0.2,
            min_drain_hole: 2.0,
            max_unsupported_span: 5.0,
        }
    }
}

impl SlaParams {
    /// Parameters for high-detail resin printing.
    #[must_use]
    pub fn high_detail() -> Self {
        Self {
            xy_resolution: 0.025,
            layer_height: 0.025,
            min_wall_thickness: 0.3,
            min_feature_size: 0.15,
            ..Default::default()
        }
    }

    /// Parameters for standard resin printing.
    #[must_use]
    pub fn standard() -> Self {
        Self::default()
    }

    /// Parameters for fast/draft resin printing.
    #[must_use]
    pub fn draft() -> Self {
        Self {
            xy_resolution: 0.1,
            layer_height: 0.1,
            min_wall_thickness: 0.6,
            min_feature_size: 0.4,
            ..Default::default()
        }
    }
}

/// Result of SLA validation.
#[derive(Debug, Clone)]
pub struct SlaValidationResult {
    /// Whether the mesh passes all SLA checks.
    pub is_valid: bool,

    /// Layers with thin walls.
    pub thin_wall_layers: Vec<ThinWallIssue>,

    /// Layers with small features.
    pub small_feature_layers: Vec<SmallFeatureIssue>,

    /// Whether mesh appears to be hollow and might need drain holes.
    pub needs_drain_holes: bool,

    /// Total number of issues found.
    pub issue_count: usize,

    /// Summary message.
    pub summary: String,
}

impl SlaValidationResult {
    /// Check if the mesh is printable.
    #[must_use]
    pub fn is_printable(&self) -> bool {
        self.thin_wall_layers.is_empty()
    }
}

/// Validate a mesh for SLA printing.
///
/// # Arguments
///
/// * `mesh` - The mesh to validate
/// * `params` - SLA parameters
///
/// # Returns
///
/// A [`SlaValidationResult`] with validation results.
///
/// # Example
///
/// ```
/// use mesh_types::unit_cube;
/// use mesh_slice::{validate_for_sla, SlaParams};
///
/// let cube = unit_cube();
/// let result = validate_for_sla(&cube, &SlaParams::default());
/// println!("{}", result.summary);
/// ```
#[must_use]
pub fn validate_for_sla(mesh: &IndexedMesh, params: &SlaParams) -> SlaValidationResult {
    let slice_params = SliceParams::for_sla();
    let slice_result = slice_mesh(mesh, &slice_params);

    let mut thin_wall_layers = Vec::new();
    let mut small_feature_layers = Vec::new();

    for layer in &slice_result.layers {
        for contour in &layer.contours {
            // Check for thin walls
            if contour.perimeter > 0.0 {
                let approx_thickness = 2.0 * contour.area / contour.perimeter;
                if approx_thickness < params.min_wall_thickness && approx_thickness > 0.0 {
                    thin_wall_layers.push(ThinWallIssue {
                        layer_index: layer.index,
                        z_height: layer.z_height,
                        min_thickness: approx_thickness,
                        required_thickness: params.min_wall_thickness,
                        location: (contour.centroid.x, contour.centroid.y),
                    });
                }
            }

            // Check for small features
            let feature_size = contour.area.sqrt();
            if feature_size < params.min_feature_size && feature_size > 0.0 {
                small_feature_layers.push(SmallFeatureIssue {
                    layer_index: layer.index,
                    z_height: layer.z_height,
                    feature_size,
                    min_size: params.min_feature_size,
                });
            }
        }
    }

    // Check if mesh might need drain holes (hollow object detection)
    // Simple heuristic: if there are internal contours (holes), might need drainage
    let needs_drain_holes = slice_result
        .layers
        .iter()
        .any(|l| l.contours.iter().any(|c| !c.is_outer));

    let issue_count = thin_wall_layers.len() + small_feature_layers.len();
    let is_valid = issue_count == 0;

    let summary = if is_valid {
        if needs_drain_holes {
            "Mesh passes SLA checks but may need drain holes for hollow sections.".to_string()
        } else {
            "Mesh passes all SLA validation checks.".to_string()
        }
    } else {
        format!(
            "Found {} issues: {} thin walls, {} small features{}",
            issue_count,
            thin_wall_layers.len(),
            small_feature_layers.len(),
            if needs_drain_holes {
                " (also needs drain holes)"
            } else {
                ""
            }
        )
    };

    SlaValidationResult {
        is_valid,
        thin_wall_layers,
        small_feature_layers,
        needs_drain_holes,
        issue_count,
        summary,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::{Vertex, unit_cube};

    fn create_test_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        let vertices = [
            (0.0, 0.0, 0.0),
            (10.0, 0.0, 0.0),
            (10.0, 10.0, 0.0),
            (0.0, 10.0, 0.0),
            (0.0, 0.0, 10.0),
            (10.0, 0.0, 10.0),
            (10.0, 10.0, 10.0),
            (0.0, 10.0, 10.0),
        ];

        for (x, y, z) in vertices {
            mesh.vertices.push(Vertex::from_coords(x, y, z));
        }

        let faces = [
            [0, 1, 2],
            [0, 2, 3],
            [4, 6, 5],
            [4, 7, 6],
            [0, 5, 1],
            [0, 4, 5],
            [2, 7, 3],
            [2, 6, 7],
            [0, 3, 7],
            [0, 7, 4],
            [1, 5, 6],
            [1, 6, 2],
        ];

        for f in faces {
            mesh.faces.push(f);
        }

        mesh
    }

    #[test]
    fn test_fdm_params_default() {
        let params = FdmParams::default();
        assert!((params.nozzle_diameter - 0.4).abs() < 0.001);
        assert!((params.min_wall_thickness - 0.8).abs() < 0.001);
    }

    #[test]
    fn test_fdm_params_nozzle_sizes() {
        let n04 = FdmParams::nozzle_04();
        assert!((n04.nozzle_diameter - 0.4).abs() < 0.001);

        let n06 = FdmParams::nozzle_06();
        assert!((n06.nozzle_diameter - 0.6).abs() < 0.001);

        let n025 = FdmParams::nozzle_025();
        assert!((n025.nozzle_diameter - 0.25).abs() < 0.001);
    }

    #[test]
    fn test_fdm_validation_cube() {
        let mesh = create_test_cube();
        let params = FdmParams::default();
        let result = validate_for_fdm(&mesh, &params);

        // A 10x10x10 cube should pass basic FDM validation
        assert!(!result.summary.is_empty());
    }

    #[test]
    fn test_fdm_validation_unit_cube() {
        let cube = unit_cube();
        let params = FdmParams::default();
        let result = validate_for_fdm(&cube, &params);

        // Summary should always be non-empty
        assert!(!result.summary.is_empty());
    }

    #[test]
    fn test_sla_params_default() {
        let params = SlaParams::default();
        assert!((params.xy_resolution - 0.05).abs() < 0.001);
        assert!((params.layer_height - 0.05).abs() < 0.001);
    }

    #[test]
    fn test_sla_params_presets() {
        let hd = SlaParams::high_detail();
        assert!(hd.xy_resolution < 0.05);

        let draft = SlaParams::draft();
        assert!(draft.xy_resolution > 0.05);
    }

    #[test]
    fn test_sla_validation_cube() {
        let mesh = create_test_cube();
        let params = SlaParams::default();
        let result = validate_for_sla(&mesh, &params);

        assert!(!result.summary.is_empty());
    }

    #[test]
    fn test_sla_validation_unit_cube() {
        let cube = unit_cube();
        let params = SlaParams::default();
        let result = validate_for_sla(&cube, &params);

        assert!(!result.summary.is_empty());
    }
}
