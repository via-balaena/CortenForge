//! Result types for template fitting operations.

use mesh_registration::RigidTransform;
use mesh_types::IndexedMesh;

/// Information about a fitting stage.
///
/// The fitting pipeline proceeds through multiple stages, each
/// tracked with diagnostic information.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum FitStage {
    /// Rigid alignment stage using ICP registration.
    RigidAlignment {
        /// RMS error after alignment.
        rms_error: f64,
        /// Number of ICP iterations performed.
        iterations: usize,
        /// Whether ICP converged.
        converged: bool,
    },

    /// Landmark-based deformation stage.
    LandmarkDeformation {
        /// Number of constraints applied.
        constraints_applied: usize,
        /// Maximum vertex displacement.
        max_displacement: f64,
    },

    /// Measurement-based adjustment stage.
    MeasurementAdjustment {
        /// Number of measurements adjusted.
        measurements_applied: usize,
        /// Maximum scale factor applied.
        max_scale_factor: f64,
    },
}

/// Result of a template fitting operation.
///
/// Contains the fitted mesh along with diagnostic information about
/// the fitting process.
///
/// # Examples
///
/// ```
/// use mesh_template::FitResult;
/// use mesh_types::{IndexedMesh, Vertex};
/// use mesh_registration::RigidTransform;
///
/// let mesh = IndexedMesh::new();
/// let result = FitResult {
///     mesh,
///     fit_error: 0.5,
///     stages: vec![],
///     transform: RigidTransform::identity(),
/// };
///
/// assert!(result.is_acceptable(1.0));
/// assert!(!result.is_acceptable(0.1));
/// ```
#[derive(Debug, Clone)]
pub struct FitResult {
    /// The fitted mesh.
    pub mesh: IndexedMesh,

    /// Overall fit error (RMS distance from targets).
    pub fit_error: f64,

    /// Stages completed during fitting.
    pub stages: Vec<FitStage>,

    /// The rigid transform applied during alignment.
    ///
    /// This is the cumulative transform from the original template
    /// position to the aligned position (before deformation).
    pub transform: RigidTransform,
}

impl FitResult {
    /// Checks if the fit error is within an acceptable threshold.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::FitResult;
    /// use mesh_types::IndexedMesh;
    /// use mesh_registration::RigidTransform;
    ///
    /// let result = FitResult {
    ///     mesh: IndexedMesh::new(),
    ///     fit_error: 0.5,
    ///     stages: vec![],
    ///     transform: RigidTransform::identity(),
    /// };
    ///
    /// assert!(result.is_acceptable(1.0));
    /// assert!(!result.is_acceptable(0.1));
    /// ```
    #[must_use]
    pub fn is_acceptable(&self, max_error: f64) -> bool {
        self.fit_error <= max_error
    }

    /// Returns the number of fitting stages completed.
    #[must_use]
    pub fn stage_count(&self) -> usize {
        self.stages.len()
    }

    /// Checks if rigid alignment was performed.
    #[must_use]
    pub fn has_rigid_alignment(&self) -> bool {
        self.stages
            .iter()
            .any(|s| matches!(s, FitStage::RigidAlignment { .. }))
    }

    /// Checks if landmark deformation was performed.
    #[must_use]
    pub fn has_landmark_deformation(&self) -> bool {
        self.stages
            .iter()
            .any(|s| matches!(s, FitStage::LandmarkDeformation { .. }))
    }

    /// Checks if measurement adjustment was performed.
    #[must_use]
    pub fn has_measurement_adjustment(&self) -> bool {
        self.stages
            .iter()
            .any(|s| matches!(s, FitStage::MeasurementAdjustment { .. }))
    }

    /// Gets the RMS error from rigid alignment, if performed.
    #[must_use]
    pub fn rigid_alignment_error(&self) -> Option<f64> {
        for stage in &self.stages {
            if let FitStage::RigidAlignment { rms_error, .. } = stage {
                return Some(*rms_error);
            }
        }
        None
    }

    /// Gets the maximum displacement from landmark deformation, if performed.
    #[must_use]
    pub fn max_landmark_displacement(&self) -> Option<f64> {
        for stage in &self.stages {
            if let FitStage::LandmarkDeformation {
                max_displacement, ..
            } = stage
            {
                return Some(*max_displacement);
            }
        }
        None
    }

    /// Gets the total number of constraints applied across all stages.
    #[must_use]
    pub fn total_constraints_applied(&self) -> usize {
        let mut total = 0;
        for stage in &self.stages {
            match stage {
                FitStage::LandmarkDeformation {
                    constraints_applied,
                    ..
                } => total += constraints_applied,
                FitStage::MeasurementAdjustment {
                    measurements_applied,
                    ..
                } => total += measurements_applied,
                FitStage::RigidAlignment { .. } => {}
            }
        }
        total
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::let_underscore_must_use)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn make_result(error: f64, stages: Vec<FitStage>) -> FitResult {
        FitResult {
            mesh: IndexedMesh::new(),
            fit_error: error,
            stages,
            transform: RigidTransform::identity(),
        }
    }

    #[test]
    fn test_is_acceptable() {
        let result = make_result(0.5, vec![]);
        assert!(result.is_acceptable(1.0));
        assert!(result.is_acceptable(0.5));
        assert!(!result.is_acceptable(0.4));
    }

    #[test]
    fn test_stage_count() {
        let result = make_result(0.0, vec![]);
        assert_eq!(result.stage_count(), 0);

        let result = make_result(
            0.0,
            vec![
                FitStage::RigidAlignment {
                    rms_error: 0.1,
                    iterations: 50,
                    converged: true,
                },
                FitStage::LandmarkDeformation {
                    constraints_applied: 5,
                    max_displacement: 0.2,
                },
            ],
        );
        assert_eq!(result.stage_count(), 2);
    }

    #[test]
    fn test_has_rigid_alignment() {
        let result = make_result(0.0, vec![]);
        assert!(!result.has_rigid_alignment());

        let result = make_result(
            0.0,
            vec![FitStage::RigidAlignment {
                rms_error: 0.1,
                iterations: 50,
                converged: true,
            }],
        );
        assert!(result.has_rigid_alignment());
    }

    #[test]
    fn test_has_landmark_deformation() {
        let result = make_result(0.0, vec![]);
        assert!(!result.has_landmark_deformation());

        let result = make_result(
            0.0,
            vec![FitStage::LandmarkDeformation {
                constraints_applied: 5,
                max_displacement: 0.2,
            }],
        );
        assert!(result.has_landmark_deformation());
    }

    #[test]
    fn test_has_measurement_adjustment() {
        let result = make_result(0.0, vec![]);
        assert!(!result.has_measurement_adjustment());

        let result = make_result(
            0.0,
            vec![FitStage::MeasurementAdjustment {
                measurements_applied: 3,
                max_scale_factor: 1.1,
            }],
        );
        assert!(result.has_measurement_adjustment());
    }

    #[test]
    fn test_rigid_alignment_error() {
        let result = make_result(0.0, vec![]);
        assert!(result.rigid_alignment_error().is_none());

        let result = make_result(
            0.0,
            vec![FitStage::RigidAlignment {
                rms_error: 0.123,
                iterations: 50,
                converged: true,
            }],
        );
        assert_relative_eq!(result.rigid_alignment_error().unwrap(), 0.123);
    }

    #[test]
    fn test_max_landmark_displacement() {
        let result = make_result(0.0, vec![]);
        assert!(result.max_landmark_displacement().is_none());

        let result = make_result(
            0.0,
            vec![FitStage::LandmarkDeformation {
                constraints_applied: 5,
                max_displacement: 1.5,
            }],
        );
        assert_relative_eq!(result.max_landmark_displacement().unwrap(), 1.5);
    }

    #[test]
    fn test_total_constraints_applied() {
        let result = make_result(0.0, vec![]);
        assert_eq!(result.total_constraints_applied(), 0);

        let result = make_result(
            0.0,
            vec![
                FitStage::RigidAlignment {
                    rms_error: 0.1,
                    iterations: 50,
                    converged: true,
                },
                FitStage::LandmarkDeformation {
                    constraints_applied: 5,
                    max_displacement: 0.2,
                },
                FitStage::MeasurementAdjustment {
                    measurements_applied: 3,
                    max_scale_factor: 1.1,
                },
            ],
        );
        assert_eq!(result.total_constraints_applied(), 8);
    }

    #[test]
    fn test_fit_stage_debug() {
        // Ensure all variants are debuggable
        let stages = [
            FitStage::RigidAlignment {
                rms_error: 0.1,
                iterations: 50,
                converged: true,
            },
            FitStage::LandmarkDeformation {
                constraints_applied: 5,
                max_displacement: 0.2,
            },
            FitStage::MeasurementAdjustment {
                measurements_applied: 3,
                max_scale_factor: 1.1,
            },
        ];

        for stage in stages {
            let _ = format!("{stage:?}");
        }
    }
}
