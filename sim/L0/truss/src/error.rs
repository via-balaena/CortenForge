//! The crate's error type.

use thiserror::Error;

/// A failure building or solving a [`Truss`](crate::Truss).
///
/// Construction errors ([`NodeIndexOutOfBounds`](Self::NodeIndexOutOfBounds),
/// [`DegenerateStrut`](Self::DegenerateStrut), …) report a malformed problem;
/// solve errors ([`SingularStiffness`](Self::SingularStiffness)) report a
/// *physically* under-supported design — a mechanism — and are how the forward
/// model fail-closes at an infeasible point rather than returning garbage.
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

    /// A strut area is not strictly positive. Areas must be `> 0`: a
    /// zero-area strut contributes no stiffness (the design-variable limit
    /// "the strut is removed" is *approached* in log-space, never reached
    /// exactly), and a negative area is unphysical.
    #[error("strut {strut} has non-positive area {area}")]
    NonPositiveArea {
        /// The offending strut index.
        strut: usize,
        /// The supplied (invalid) area.
        area: f64,
    },

    /// The assembled stiffness matrix is not positive-definite over the free
    /// degrees of freedom, so `K·u = f` has no unique solution. Physically the
    /// design is a **mechanism**: some free node can displace with zero strain
    /// energy (too few struts, all-collinear struts at a joint, or a subset of
    /// areas driven so small the structure has effectively come apart). This is
    /// the feasibility boundary of the forward model, reported rather than
    /// producing a meaningless displacement.
    #[error("stiffness matrix is singular over the free DOFs (the design is a mechanism)")]
    SingularStiffness,
}
