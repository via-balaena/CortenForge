//! The crate's error type.

use thiserror::Error;

/// A failure building or solving a [`Truss`](crate::Truss).
///
/// Construction errors ([`NodeIndexOutOfBounds`](Self::NodeIndexOutOfBounds),
/// [`DegenerateStrut`](Self::DegenerateStrut), …) report a malformed problem;
/// solve errors ([`SingularStiffness`](Self::SingularStiffness)) report a design
/// whose stiffness is not positive-definite — typically an under-supported
/// *mechanism* — and are how the forward model fail-closes at an infeasible
/// point rather than returning garbage.
#[derive(Debug, Clone, PartialEq, Error)]
#[non_exhaustive]
pub enum TrussError {
    /// A strut, support, or load names a node index `≥ n_nodes`.
    #[error("node index {index} out of bounds (n_nodes = {n_nodes})")]
    NodeIndexOutOfBounds {
        /// The offending node index.
        index: usize,
        /// The number of nodes in the truss.
        n_nodes: usize,
    },

    /// A strut connects a node to itself (`i == j`), which has no length or
    /// direction.
    #[error("strut {strut} connects node {node} to itself")]
    SelfLoopStrut {
        /// The offending strut index.
        strut: usize,
        /// The repeated node index.
        node: usize,
    },

    /// A strut's two endpoints coincide, so it has zero length and no defined
    /// axis.
    #[error("strut {strut} is degenerate (its endpoints coincide, length = 0)")]
    DegenerateStrut {
        /// The offending strut index.
        strut: usize,
    },

    /// The supplied area slice does not have one entry per strut.
    #[error("expected {expected} areas (one per strut), got {got}")]
    AreaCountMismatch {
        /// The number of struts.
        expected: usize,
        /// The length of the supplied area slice.
        got: usize,
    },

    /// A strut area is not finite and strictly positive. Areas must be a finite
    /// `> 0`: a zero-area strut contributes no stiffness (the design-variable
    /// limit "the strut is removed" is *approached* in log-space, never reached
    /// exactly), a negative area is unphysical, and a non-finite area (`±∞` /
    /// `NaN`) would poison the stiffness matrix.
    #[error("strut {strut} has invalid area {area} (must be finite and > 0)")]
    InvalidArea {
        /// The offending strut index.
        strut: usize,
        /// The supplied (invalid) area.
        area: f64,
    },

    /// A strut's Young's modulus is not finite and strictly positive. Like the
    /// area it scales the strut's stiffness directly, so a non-positive `E`
    /// makes the assembled matrix indefinite (mis-reported as a mechanism) and a
    /// non-finite `E` poisons it — both are rejected at construction.
    #[error("strut {strut} has invalid Young's modulus {youngs_modulus} (must be finite and > 0)")]
    InvalidModulus {
        /// The offending strut index.
        strut: usize,
        /// The supplied (invalid) modulus.
        youngs_modulus: f64,
    },

    /// The Cholesky factorization of the free-DOF stiffness matrix failed, so
    /// `K·u = f` has no usable solution. Physically this is almost always a
    /// **mechanism**: some free node can displace with zero strain energy (too
    /// few struts, all-collinear struts at a joint, or a subset of areas driven
    /// so small the structure has effectively come apart). The factorization
    /// tests numerical positive-definiteness, so a genuinely positive-definite
    /// but extremely ill-conditioned `K` right at the mechanism boundary can
    /// also trip it — the honest reading is "not (numerically) positive-definite",
    /// with a mechanism the dominant cause. Either way it is the feasibility
    /// boundary of the forward model, reported rather than producing a
    /// meaningless displacement.
    #[error(
        "stiffness matrix is not positive-definite over the free DOFs (the design is a mechanism)"
    )]
    SingularStiffness,
}
