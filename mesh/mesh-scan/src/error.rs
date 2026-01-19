//! Error types for scan processing operations.

use std::fmt;

/// Result type for scan processing operations.
pub type ScanResult<T> = Result<T, ScanError>;

/// Errors that can occur during scan processing.
#[derive(Debug)]
pub enum ScanError {
    /// Point cloud is empty.
    EmptyPointCloud,

    /// Not enough points for the requested operation.
    InsufficientPoints {
        /// Minimum number of points required.
        required: usize,
        /// Actual number of points provided.
        actual: usize,
    },

    /// Invalid parameter value.
    InvalidParameter {
        /// Description of why the parameter is invalid.
        reason: String,
    },

    /// I/O error during file operations.
    IoError(std::io::Error),

    /// Unsupported file format.
    UnsupportedFormat {
        /// The format that was not supported.
        format: String,
    },

    /// PLY file parsing error.
    PlyParseError {
        /// Description of the parsing error.
        reason: String,
    },

    /// Surface reconstruction failed.
    ReconstructionFailed {
        /// Description of why reconstruction failed.
        reason: String,
    },

    /// Multi-scan alignment failed.
    AlignmentFailed {
        /// Description of why alignment failed.
        reason: String,
    },

    /// Error from mesh registration operations.
    RegistrationError(mesh_registration::RegistrationError),

    /// Error from mesh repair operations.
    RepairError(mesh_repair::RepairError),

    /// Mesh is empty.
    EmptyMesh,

    /// Normal estimation failed.
    NormalEstimationFailed {
        /// Description of why normal estimation failed.
        reason: String,
    },
}

impl fmt::Display for ScanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyPointCloud => write!(f, "point cloud is empty"),
            Self::InsufficientPoints { required, actual } => {
                write!(
                    f,
                    "insufficient points: need at least {required}, got {actual}"
                )
            }
            Self::InvalidParameter { reason } => write!(f, "invalid parameter: {reason}"),
            Self::IoError(e) => write!(f, "I/O error: {e}"),
            Self::UnsupportedFormat { format } => write!(f, "unsupported format: {format}"),
            Self::PlyParseError { reason } => write!(f, "PLY parsing error: {reason}"),
            Self::ReconstructionFailed { reason } => write!(f, "reconstruction failed: {reason}"),
            Self::AlignmentFailed { reason } => write!(f, "alignment failed: {reason}"),
            Self::RegistrationError(e) => write!(f, "registration error: {e}"),
            Self::RepairError(e) => write!(f, "repair error: {e}"),
            Self::EmptyMesh => write!(f, "mesh is empty"),
            Self::NormalEstimationFailed { reason } => {
                write!(f, "normal estimation failed: {reason}")
            }
        }
    }
}

impl std::error::Error for ScanError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::IoError(e) => Some(e),
            Self::RegistrationError(e) => Some(e),
            Self::RepairError(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for ScanError {
    fn from(err: std::io::Error) -> Self {
        Self::IoError(err)
    }
}

impl From<mesh_registration::RegistrationError> for ScanError {
    fn from(err: mesh_registration::RegistrationError) -> Self {
        Self::RegistrationError(err)
    }
}

impl From<mesh_repair::RepairError> for ScanError {
    fn from(err: mesh_repair::RepairError) -> Self {
        Self::RepairError(err)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_point_cloud_error() {
        let err = ScanError::EmptyPointCloud;
        assert_eq!(format!("{err}"), "point cloud is empty");
    }

    #[test]
    fn test_insufficient_points_error() {
        let err = ScanError::InsufficientPoints {
            required: 10,
            actual: 5,
        };
        assert_eq!(
            format!("{err}"),
            "insufficient points: need at least 10, got 5"
        );
    }

    #[test]
    fn test_invalid_parameter_error() {
        let err = ScanError::InvalidParameter {
            reason: "voxel size must be positive".to_string(),
        };
        assert_eq!(
            format!("{err}"),
            "invalid parameter: voxel size must be positive"
        );
    }

    #[test]
    fn test_unsupported_format_error() {
        let err = ScanError::UnsupportedFormat {
            format: "abc".to_string(),
        };
        assert_eq!(format!("{err}"), "unsupported format: abc");
    }

    #[test]
    fn test_ply_parse_error() {
        let err = ScanError::PlyParseError {
            reason: "invalid header".to_string(),
        };
        assert_eq!(format!("{err}"), "PLY parsing error: invalid header");
    }

    #[test]
    fn test_reconstruction_failed_error() {
        let err = ScanError::ReconstructionFailed {
            reason: "no valid triangles".to_string(),
        };
        assert_eq!(
            format!("{err}"),
            "reconstruction failed: no valid triangles"
        );
    }

    #[test]
    fn test_alignment_failed_error() {
        let err = ScanError::AlignmentFailed {
            reason: "ICP did not converge".to_string(),
        };
        assert_eq!(format!("{err}"), "alignment failed: ICP did not converge");
    }

    #[test]
    fn test_empty_mesh_error() {
        let err = ScanError::EmptyMesh;
        assert_eq!(format!("{err}"), "mesh is empty");
    }

    #[test]
    fn test_normal_estimation_failed_error() {
        let err = ScanError::NormalEstimationFailed {
            reason: "not enough neighbors".to_string(),
        };
        assert_eq!(
            format!("{err}"),
            "normal estimation failed: not enough neighbors"
        );
    }

    #[test]
    fn test_io_error_conversion() {
        let io_err = std::io::Error::new(std::io::ErrorKind::NotFound, "file not found");
        let scan_err: ScanError = io_err.into();
        assert!(matches!(scan_err, ScanError::IoError(_)));
    }
}
