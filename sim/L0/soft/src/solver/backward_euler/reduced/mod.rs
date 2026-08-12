//! Reduced-basis machinery — rung **R1** of `docs/SIM_SOFT_R1_REDUCED_BASIS_PLAN.md`,
//! built in three deliberately separable pieces:
//!
//! | | question it answers | types |
//! |---|---|---|
//! | **R1.0** | is this material's deformation low-dimensional? | [`SnapshotSet`], [`PodBasis`] |
//! | **R1.1** | does a Galerkin solve in that subspace track the basis's own ceiling? | [`ReducedNewtonSolver`], [`ReducedStep`] |
//! | **R1.2** | what is the gradient of that solve, with `Φ` held constant? | [`ReducedAdjoint`] |
//!
//! The split is what makes each answer worth having: R1.0 could kill the whole
//! reduced-order ladder before any solver existed, and R1.1's error is meaningless
//! without R1.0's separately-measured floor to divide it by.
//!
//! ## What the basis is, and what it is measured against
//!
//! `Φ` spans the solver's **free** DOFs (pinned and roller-constrained DOFs are held at
//! their `x_prev` values and are not unknowns). A displacement `u = x − x_rest` is
//! approximated `u ≈ Φq`, and [`PodBasis::projection_error`] reports
//! `‖u − ΦΦᵀu‖ / ‖u‖` — the basis's **ceiling**, since no reduced solve can do better
//! than the best `q`.
//!
//! ⚠ **A projection error measured on the trajectory the basis was trained on means
//! nothing** — the basis reproduces its own training data almost by construction. Every
//! gate in the plan is stated on held-out trajectories, and this module is deliberately
//! free of any "fit and score" convenience that would make the mistake easy.
//!
//! ## Cost
//!
//! `fit` builds the `m × m` Gram matrix (`m` = snapshot count, `m ≪ n_free`) and
//! eigendecomposes it, which is `O(n·m² + m³)`. The `SIGMA_FLOOR_REL` constant in the
//! private `pod` module documents the precision this route costs, and why it is
//! acceptable here.

mod newton;
mod pod;
mod sensitivity;
mod snapshot;

pub use newton::{ReducedNewtonSolver, ReducedStep};
pub use pod::{Inner, PodBasis, PodError};
pub use sensitivity::ReducedAdjoint;
pub use snapshot::SnapshotSet;
