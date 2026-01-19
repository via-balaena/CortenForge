//! Error types for lattice generation.

use thiserror::Error;

/// Errors that can occur during lattice generation.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum LatticeError {
    /// The bounding box is invalid (min >= max in some dimension).
    #[error("invalid bounding box: min {min:?} >= max {max:?} in at least one dimension")]
    InvalidBounds {
        /// Minimum corner of the bounds.
        min: [f64; 3],
        /// Maximum corner of the bounds.
        max: [f64; 3],
    },

    /// The cell size is too small or non-positive.
    #[error("cell size must be positive, got {0}")]
    InvalidCellSize(f64),

    /// The strut thickness is invalid.
    #[error(
        "strut thickness must be positive and less than cell size, got thickness={thickness}, cell_size={cell_size}"
    )]
    InvalidStrutThickness {
        /// The provided strut thickness.
        thickness: f64,
        /// The cell size for reference.
        cell_size: f64,
    },

    /// The density value is out of range.
    #[error("density must be in range [0.0, 1.0], got {0}")]
    InvalidDensity(f64),

    /// The resolution is too low.
    #[error("resolution must be at least 2, got {0}")]
    InvalidResolution(usize),

    /// The shell thickness is invalid.
    #[error("shell thickness must be non-negative, got {0}")]
    InvalidShellThickness(f64),

    /// The input mesh is empty.
    #[error("input mesh is empty (no vertices or faces)")]
    EmptyMesh,

    /// The input mesh is not watertight.
    #[error("mesh must be watertight for infill generation")]
    NonWatertightMesh,

    /// Failed to compute SDF for the mesh.
    #[error("failed to compute signed distance field: {0}")]
    SdfError(String),

    /// The interior volume is too small for lattice generation.
    #[error("interior volume too small for lattice generation after shell offset")]
    InteriorTooSmall,
}
