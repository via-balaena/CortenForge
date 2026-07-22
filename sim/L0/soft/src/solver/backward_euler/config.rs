//! Solver configuration + friction-gradient output data types for
//! [`CpuNewtonSolver`](super::CpuNewtonSolver).

use crate::Vec3;
use crate::mesh::VertexId;
use crate::solver::lm::LmConfig;

/// Solver configuration — integration parameters the skeleton scene
/// (spec §2) consumes. `skeleton()` returns the spec-§2 defaults.
#[derive(Clone, Copy, Debug)]
pub struct SolverConfig {
    /// Integration time-step (seconds).
    pub dt: f64,
    /// Newton residual tolerance on the free-DOF residual.
    pub tol: f64,
    /// Reference-configuration mass density (`kg/m^3`). The solver
    /// derives per-DOF lumped mass from this — see `reference_geometry`.
    pub density: f64,
    /// Maximum Newton iterations before declaring divergence.
    pub max_newton_iter: usize,
    /// Maximum Armijo backtracks before declaring line-search stall.
    pub max_line_search_backtracks: usize,
    /// Body-force gravitational acceleration along `+ẑ` (`m/s²`).
    /// Pass a negative value for "downward" gravity (e.g. `-9.81`).
    /// Wired alongside `tests/contact_drop_rest.rs` (drop-and-rest
    /// gravity hygiene); scalar (not `Vec3`) form preserves
    /// [`Self::skeleton`]'s `const fn` signature. Default `0.0` keeps
    /// pre-gravity regression nets bit-equal —
    /// `assemble_external_force` short-circuits the body-force scatter
    /// when this is exactly zero.
    pub gravity_z: f64,
    /// Levenberg-Marquardt regularization for non-PD tangent rescue
    /// per `docs/F3_LM_REGULARIZATION_SPEC.md`. `None` (the
    /// [`Self::skeleton`] default) preserves pre-F3 behavior bit-equal
    /// via `LmState::disabled` (pub(super) — see `super::lm`) short-circuit
    /// at `factor_free_tangent`'s retry loop: `Llt` first, then direct
    /// `Lu` fallback on non-PD, no `+λI`. `Some(LmConfig)` activates
    /// the in-iter Marquardt adapter; the `Lu` fallback then becomes
    /// the λ-saturation surface. Fork-B (cf-sim-research) consumers
    /// opt in via [`LmConfig::fork_b`] paired with
    /// [`Solver::try_step`](crate::solver::Solver::try_step) for graceful
    /// failure on Armijo stall.
    pub lm_regularization: Option<LmConfig>,
    /// Coulomb friction coefficient `μ_c` for the smoothed-Coulomb friction term
    /// (`contact::friction`). Default `0.0` = FRICTIONLESS, which short-circuits the
    /// friction scatter in the forward assembly → bit-equal to the pre-friction path
    /// (the [`Self::skeleton`] / `gravity_z = 0` pattern). Friction enters the FORWARD
    /// Newton solve (residual + its Hessian); the differentiable tangent
    /// (`factor_at_position`) stays friction-free until the differentiability leaf.
    ///
    /// PR1 is FORWARD-ONLY: gradients with `friction_mu > 0` are **not** supported and the
    /// differentiable paths (`step`, the VJP / equilibrium-sensitivity methods) panic rather
    /// than silently return a tangent that omits the friction Hessian. Use `replay_step` for
    /// forward-only friction; PR2 (the differentiability leaf) wires friction into the adjoint.
    pub friction_mu: f64,
    /// Friction velocity threshold `ε_v` (m/s): the transition-zone width in displacement
    /// space is `w = dt·ε_v` (below this sliding speed the smoothed force ramps from zero
    /// — the stick regime). Only consulted when `friction_mu > 0`. IPC default
    /// `≈ 1e-3·L_bbox` m/s.
    pub friction_eps_v: f64,
    /// Nodal-averaged **F-bar** volumetric-locking cure (Part 2 Ch 05 02-f-bar.md).
    /// Default `false` = plain per-element Tet4, **bit-equal** to the pre-F-bar
    /// path (the elastic assembly short-circuits to the unmodified per-element
    /// loop). `true` feeds the constitutive law the patch-modified kinematic
    /// `F* = (J̄/J)^{1/3} F`, curing the `ν → 0.5` over-stiffening that pins the
    /// standalone Tet4 gates to `ν = 0.4`. Enables the `ν ≤ 0.49` regime
    /// (Ecoflex 00-30's real Poisson ratio); above `ν = 0.49` mixed-u-p is the
    /// spec's recommended cure instead.
    ///
    /// PR1 is FORWARD-ONLY: the differentiable paths (`step`, the VJP /
    /// sensitivity methods) **panic** when `fbar` is set rather than silently
    /// return a tangent that omits the F-bar neighbor coupling. Use
    /// `replay_step` for forward-only F-bar; the differentiability leaf (PR2)
    /// wires the coupling into the adjoint. (Mirrors the `friction_mu`
    /// forward-only-in-PR1 contract above.)
    pub fbar: bool,
}

impl SolverConfig {
    /// Scope §2 defaults for the walking-skeleton scene: `dt = 1e-2`,
    /// `tol = 1e-10` (five digits below gradcheck's 1e-5 bar),
    /// `density = 1030` (silicone-class), up to 10 Newton iterations +
    /// 20 backtracks per iteration. `gravity_z = 0` — the
    /// drop-and-rest fixture opts in by mutating the field on a
    /// constructed config (mirrors the Hertzian and compressive-block
    /// fixtures' `STATIC_DT` bumping pattern).
    #[must_use]
    pub const fn skeleton() -> Self {
        Self {
            dt: 1e-2,
            tol: 1e-10,
            density: 1030.0,
            max_newton_iter: 10,
            max_line_search_backtracks: 20,
            gravity_z: 0.0,
            lm_regularization: None,
            friction_mu: 0.0,
            friction_eps_v: 0.0,
            fbar: false,
        }
    }
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self::skeleton()
    }
}

/// The friction reaction force on the rigid collider along a chosen direction, plus its
/// first-order sensitivities — the tangential-grip readout a staggered coupling routes
/// onto its rigid-state tape.
///
/// Produced by [`CpuNewtonSolver::friction_reaction_gradients`](super::CpuNewtonSolver::friction_reaction_gradients) (which documents the
/// math). All fields are along the `react_dir` the gradients were built with; `dforce_dx`
/// is length `n_dof` (zeros off the active set), the rest scalar.
#[derive(Clone, Debug)]
pub struct FrictionReactionGradients {
    /// `F = (Σ_v ∇D_v)·react_dir` — the reaction force along `react_dir`.
    pub force: f64,
    /// `∂F/∂x*` (length `n_dof`) — frozen-lag `∇²D` slip term plus the normal-force
    /// (λⁿ) coupling.
    pub dforce_dx: Vec<f64>,
    /// `∂F/∂x_prev` (length `n_dof`) — the friction reference `x_start = x_prev + Δ_surf`
    /// makes the reaction depend on the step-start config too: `∂F/∂x_prev = −`(the
    /// frozen-lag `∇²D` slip term of [`Self::dforce_dx`], λⁿ-coupling excluded, since λⁿ
    /// tracks `x*` not `x_prev`). The state-config companion of [`Self::dforce_ddrift`].
    pub dforce_dxprev: Vec<f64>,
    /// `∂F/∂Δ_surf` along the build's `drift_dir` (the moving-collider reference shift,
    /// λ-independent).
    pub dforce_ddrift: f64,
    /// `∂F/∂height` along the build's `pose_dir` plane translation (the λ-coupling).
    pub dforce_dheight: f64,
}

/// The per-vertex friction force on the rigid collider `∇D_v` (a 3-vector) and its
/// first-order sensitivities.
///
/// The VECTOR, per-contacted-vertex generalization of [`FrictionReactionGradients`] (which
/// projects onto a single `react_dir` and sums). A staggered coupling needs the per-vertex
/// VECTOR force to assemble the off-COM friction MOMENT `Σ_v (r_v − c) × ∇D_v` and its
/// Jacobian; the scalar aggregate cannot (the moment arm `r_v − c` weights each vertex
/// differently).
///
/// `force = ∇D_v` is the reaction on the RIGID body at vertex `v` (the soft body feels
/// `−∇D_v`). `dforce_dx`/`dforce_dxprev` are row-major `3 × n_dof` blocks (`∂force[r]/∂x[col]`
/// at flat index `r·n_dof + col`); the rest are `3`-vectors. Produced by
/// [`CpuNewtonSolver::friction_force_jacobians`](super::CpuNewtonSolver::friction_force_jacobians).
#[derive(Clone, Debug)]
pub struct FrictionVertexForce {
    /// The contacted vertex.
    pub vid: VertexId,
    /// `∇D_v` — the friction force on the rigid body at `v` (on the soft body: `−∇D_v`).
    pub force: Vec3,
    /// `∂force/∂x*` — row-major `3 × n_dof` (frozen-lag `∇²D_v` slip at `v`'s own coords plus
    /// the normal-force λⁿ coupling `a_v ⊗ ∂λⁿ_v/∂x_c` spread over the contact-neighbor coords).
    pub dforce_dx: Vec<f64>,
    /// `∂force/∂x_prev` — row-major `3 × n_dof`; `−`(the frozen-lag `∇²D_v` slip at `v`'s coords),
    /// λⁿ-coupling excluded (λⁿ tracks `x*`, not `x_prev`), via `x_start = x_prev + Δ_surf`.
    pub dforce_dxprev: Vec<f64>,
    /// `∂force/∂Δ_surf` along the build's `drift_dir` (`−∇²D_v·drift_dir`, the moving-collider
    /// reference shift, λ-independent).
    pub dforce_ddrift: Vec3,
    /// `∂force/∂height` along the build's `pose_dir` plane translation (`a_v·(n̂·∂(plane)/∂pose)`,
    /// the λⁿ coupling).
    pub dforce_dheight: Vec3,
    /// `∂force/∂μ_c` — the friction force is `∇D_v = μ_c·λⁿ_v·f₁·Tû`, LINEAR in the Coulomb
    /// coefficient `μ_c`, so `∂force/∂μ_c = ∇D_v/μ_c` (the DIRECT channel, at fixed `x*`/λⁿ). The
    /// dominant lever for the friction-coefficient co-design gradient (the soft `x*` channel is
    /// tiny in deep slip).
    pub dforce_dmu_c: Vec3,
}
