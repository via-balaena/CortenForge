//! `CpuNewtonSolver` — backward-Euler Newton with Armijo line-search.
//!
//! Phase B fills in:
//! - residual \(r(x; \theta) = M(x - \hat{x})/\Delta t^2 + f_\text{int} -
//!   f_\text{ext}\),
//! - tangent assembly into sparse CSR,
//! - faer `SymbolicLlt::try_new` (once per pattern) +
//!   `Llt::try_new_with_symbolic` (per re-factor),
//! - Newton + Armijo (spec §5 R-1),
//! - push of `NewtonStepVjp` onto the chassis tape via
//!   `Tape::push_custom`.

use sim_ml_chassis::Tensor;

use super::{CpuTape, NewtonStep, Solver};
use crate::contact::ContactModel;
use crate::element::Element;
use crate::material::Material;
use crate::mesh::Mesh;

/// Solver configuration — time step and Newton tolerance. Phase B
/// wires the spec §2 skeleton-scene defaults (`dt = 1e-2`).
#[derive(Clone, Copy, Debug, Default)]
pub struct SolverConfig {
    /// Integration time-step (seconds).
    pub dt: f64,
    /// Newton residual tolerance.
    pub tol: f64,
}

/// CPU backward-Euler Newton solver.
///
/// Six generic parameters: material `M`, element `E<N, G>`, mesh `Msh`,
/// contact `C`, and const-generic `(N, G)` for element shape.
/// Monomorphized per skeleton type alias `SkeletonSolver`.
pub struct CpuNewtonSolver<M, E, Msh, C, const N: usize, const G: usize>
where
    M: Material,
    E: Element<N, G>,
    Msh: Mesh,
    C: ContactModel,
{
    /// Material model.
    pub material: M,
    /// Element shape.
    pub element: E,
    /// Mesh storage.
    pub mesh: Msh,
    /// Contact model.
    pub contact: C,
    /// Integration configuration.
    pub config: SolverConfig,
}

impl<M, E, Msh, C, const N: usize, const G: usize> Solver for CpuNewtonSolver<M, E, Msh, C, N, G>
where
    M: Material,
    E: Element<N, G>,
    Msh: Mesh,
    C: ContactModel,
{
    type Tape = CpuTape;

    fn step(
        &mut self,
        _tape: &mut Self::Tape,
        _x_prev: &Tensor<f64>,
        _v_prev: &Tensor<f64>,
        _theta: &Tensor<f64>,
        _dt: f64,
    ) -> NewtonStep<Self::Tape> {
        unimplemented!("skeleton phase 2")
    }

    fn replay_step(
        &self,
        _x_prev: &Tensor<f64>,
        _v_prev: &Tensor<f64>,
        _theta: &Tensor<f64>,
        _dt: f64,
    ) -> NewtonStep<Self::Tape> {
        unimplemented!("skeleton phase 2")
    }

    fn current_dt(&self) -> f64 {
        unimplemented!("skeleton phase 2")
    }

    fn convergence_tol(&self) -> f64 {
        unimplemented!("skeleton phase 2")
    }
}
