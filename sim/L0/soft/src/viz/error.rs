//! Error surface for the viz primitives.

/// Error surface for viz primitives.
#[derive(Clone, Debug)]
pub enum VizError {
    /// A per-tet scalar's length does not equal `mesh.n_tets()`.
    PerTetScalarLengthMismatch {
        /// Name of the offending scalar (`BTreeMap` key).
        name: String,
        /// Required length (= `mesh.n_tets()`).
        expected: usize,
        /// Length actually supplied.
        actual: usize,
    },
    /// Cutting plane axis index out of range (must be 0/1/2 = x/y/z).
    InvalidPlaneAxis {
        /// Bad axis value supplied.
        axis: usize,
    },
    /// Marching-squares grid resolution must be strictly positive.
    InvalidResolution {
        /// Bad resolution value supplied (in plane units, m).
        value: f64,
    },
    /// A per-vertex displacement field's length does not equal
    /// `analysis_mesh.n_vertices()`.
    PerVertexLengthMismatch {
        /// Required length (= `analysis_mesh.n_vertices()`).
        expected: usize,
        /// Length actually supplied.
        actual: usize,
    },
}

impl std::fmt::Display for VizError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::PerTetScalarLengthMismatch {
                name,
                expected,
                actual,
            } => write!(
                f,
                "per-tet scalar `{name}` has length {actual}, expected {expected} (= mesh.n_tets())"
            ),
            Self::InvalidPlaneAxis { axis } => {
                write!(f, "plane axis must be 0/1/2 (x/y/z), got {axis}")
            }
            Self::InvalidResolution { value } => {
                write!(f, "marching-squares resolution must be > 0, got {value}")
            }
            Self::PerVertexLengthMismatch { expected, actual } => write!(
                f,
                "per-vertex field has length {actual}, expected {expected} \
                 (= analysis_mesh.n_vertices())"
            ),
        }
    }
}

impl std::error::Error for VizError {}
