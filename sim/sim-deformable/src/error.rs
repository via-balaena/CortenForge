//! Error types for deformable body simulation.

use thiserror::Error;

/// Errors that can occur during deformable body simulation.
#[derive(Debug, Error)]
pub enum DeformableError {
    /// Invalid mesh topology (e.g., degenerate triangles, disconnected components).
    #[error("Invalid mesh topology: {0}")]
    InvalidTopology(String),

    /// Invalid material parameters (e.g., negative stiffness).
    #[error("Invalid material: {0}")]
    InvalidMaterial(String),

    /// Index out of bounds.
    #[error("Index out of bounds: {0}")]
    IndexOutOfBounds(String),

    /// Constraint is invalid (e.g., degenerate constraint).
    #[error("Invalid constraint: {0}")]
    InvalidConstraint(String),

    /// Solver failed to converge.
    #[error("Solver did not converge: {0}")]
    SolverFailure(String),

    /// Configuration error.
    #[error("Invalid configuration: {0}")]
    InvalidConfig(String),

    /// Numerical error (`NaN`, infinity).
    #[error("Numerical error: {0}")]
    NumericalError(String),
}

impl DeformableError {
    /// Create an invalid topology error.
    pub fn invalid_topology(msg: impl Into<String>) -> Self {
        Self::InvalidTopology(msg.into())
    }

    /// Create an invalid material error.
    pub fn invalid_material(msg: impl Into<String>) -> Self {
        Self::InvalidMaterial(msg.into())
    }

    /// Create an index out of bounds error.
    pub fn index_out_of_bounds(msg: impl Into<String>) -> Self {
        Self::IndexOutOfBounds(msg.into())
    }

    /// Create an invalid constraint error.
    pub fn invalid_constraint(msg: impl Into<String>) -> Self {
        Self::InvalidConstraint(msg.into())
    }

    /// Create a solver failure error.
    pub fn solver_failure(msg: impl Into<String>) -> Self {
        Self::SolverFailure(msg.into())
    }

    /// Create an invalid config error.
    pub fn invalid_config(msg: impl Into<String>) -> Self {
        Self::InvalidConfig(msg.into())
    }

    /// Create a numerical error.
    pub fn numerical_error(msg: impl Into<String>) -> Self {
        Self::NumericalError(msg.into())
    }
}

/// Result type for deformable body operations.
pub type Result<T> = std::result::Result<T, DeformableError>;
