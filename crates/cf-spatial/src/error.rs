//! Error types for spatial operations.

use crate::VoxelCoord;

/// Errors that can occur during spatial operations.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum SpatialError {
    /// The voxel size must be positive.
    #[error("voxel size must be positive, got {0}")]
    InvalidVoxelSize(f64),

    /// A coordinate is out of the valid range for grid operations.
    #[error("coordinate {coord:?} is out of bounds")]
    OutOfBounds {
        /// The coordinate that was out of bounds.
        coord: VoxelCoord,
    },

    /// The grid dimensions are invalid.
    #[error("invalid grid dimensions: {width}x{height}x{depth}")]
    InvalidDimensions {
        /// Width dimension.
        width: usize,
        /// Height dimension.
        height: usize,
        /// Depth dimension.
        depth: usize,
    },

    /// Integer overflow occurred during coordinate calculation.
    #[error("integer overflow during coordinate calculation")]
    IntegerOverflow,
}
