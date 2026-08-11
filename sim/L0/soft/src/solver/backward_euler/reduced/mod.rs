//! Reduced-basis machinery — rung **R1.0** of `docs/SIM_SOFT_R1_REDUCED_BASIS_PLAN.md`.
//!
//! Snapshots and a POD basis. **No reduced solver** — that is R1.1, and the split is
//! deliberate: R1.0 answers *is this material's deformation low-dimensional?* and can
//! kill the whole reduced-order ladder before any solver code exists.
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

mod pod;
mod snapshot;

pub use pod::{Inner, PodBasis, PodError};
pub use snapshot::SnapshotSet;
