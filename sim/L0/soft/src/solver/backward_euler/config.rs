//! Solver configuration + friction-gradient output data types for
//! [`CpuNewtonSolver`](super::CpuNewtonSolver).

use crate::Vec3;
use crate::mesh::VertexId;
use crate::solver::lm::LmConfig;

/// Where Newton starts each step — the initial iterate `x⁰` handed to
/// [`CpuNewtonSolver`](super::CpuNewtonSolver)'s outer loop.
///
/// Backward Euler's free-DOF residual is
///
/// ```text
///     r(x) = (m/Δt²)·(x − x̂) + f_int(x) − f_ext,     x̂ = x_prev + Δt·v_prev
/// ```
///
/// so the choice here is *which term the first iterate already satisfies*.
/// It changes the PATH Newton walks, never the root it walks to: every
/// variant converges to the same `‖r‖ < tol` state, so this is a cost knob,
/// not a physics knob.
///
/// ⚠ **It is not free of risk.** A guess further from the root can land
/// outside the convergence basin, cost *more* iterations, or — under an
/// [`IpcRigidContact`](crate::IpcRigidContact) log barrier — overshoot into
/// penetration, where the barrier energy is infinite. Newton's intermediate
/// iterates are not validity-checked — only the step-start state and the
/// converged state are (Phase 4 Decision Q's two boundaries) — so an inverted
/// first iterate surfaces as a stall or a non-finite residual rather than a
/// clean [`SolverFailure::ValidityViolation`](crate::SolverFailure).
/// Measure per fixture before switching.
///
/// `#[non_exhaustive]` because this set may still grow. `InertialWithLoad`
/// measured **32.7× worse** on a stiff fixture (recon §2f), so it cannot be an
/// unconditional default.
///
/// ⛔ **The obvious rescue has been tried and killed.** A per-step selector
/// ranking the candidates by `‖r‖` was built, measured on all six cells, and
/// removed: it picks the WORST arm on the large-deflection cantilever (4.7× on
/// iterations), because `‖r(x⁰)‖` is not a proxy for distance to the root.
/// **Read recon §2g before proposing any variant of that idea.**
///
/// ⛔ **A second rescue has been built, measured and killed: mesh smoothing of
/// the predicted displacement.** `tests/stick_impact.rs` establishes that on an
/// impact frame the *shape* of the guess error, not its distance from the root,
/// sets the iteration count — a band-localised error costs **`46.8×`** a smooth
/// one at matched distance (`whether_the_shape_of_the_guess_error_sets_the_
/// iteration_count`, which computes and gates that ratio) — and that replacing
/// `Δt·v_prev` with a smooth tip-matched profile cuts `p99` frame cost by
/// **`~6×`** (`whether_a_smoothed_start_moves_the_p99_end_to_end`). That result
/// is real, and it is **beam specific**: the profile delivering it is a linear
/// axial ramp, which knows the fixture is a cantilever.
///
/// Two mesh-general forms were implemented as a `SmoothedInertial` variant and
/// both lost to plain [`Self::Inertial`] on the same fixture: **graph-Laplacian
/// diffusion with a peak rescale**, which decays the field *away* from the
/// impulse instead of ramping *toward* it and inverted elements at iteration 0;
/// and **harmonic extension from the driven nodes**, which was valid and
/// trajectory-exact and still slower than doing nothing.
///
/// ⚠⚠ **Those two arms are NOT in the tree and were never committed**, so the
/// figures they produced are unrecoverable and are deliberately not quoted here.
/// What is quoted above is what `tests/stick_impact.rs` can still reproduce on
/// demand. Treat the two failures as directional evidence, not as measurements
/// you can check — and if you want numbers, re-run the experiment rather than
/// trusting a doc comment about code nobody can read.
///
/// ★ Two findings from that attempt DO survive, because they were established by
/// difference rather than by the numbers:
///
/// - **A midside node is not an ordinary graph vertex.** Neighbour-averaging one
///   walks it off the edge it bisects and folds the quadratic element. The same
///   code never failed on Tet4, which has no midsides.
/// - **"Smooth" is not the property that matters.** A harmonic extension of the
///   predicted field is smooth and still lost to the ramp on the same equation,
///   so the win comes from imposing a specific *low-strain global shape*, which
///   graph smoothing does not produce.
///
/// ⛔ **Modal projection was that candidate, and it has now been built,
/// measured and set down too** — `tests/stick_impact.rs` Probe 4, which unlike
/// the two arms above is IN THE TREE and re-runnable. Projecting `Δt·v_prev`
/// onto the leading modes of a ring-down POD basis is worth `25 → 6` iterations
/// at the game strike, beats a norm-matched scalar shrink by `6.8×` and a
/// rank-matched random subspace by `7.5×`, and beats random at every magnitude
/// and every rank. The mechanism is real and it is the modes.
///
/// It still loses to the beam-specific ramp at every magnitude (`6` against
/// `4` at the game strike, `29` against `25` at `8×`), so it does not clear the
/// bar and is not offered as a variant here. Three findings from it are worth
/// carrying:
///
/// - **A predictor cannot be wrong, only expensive.** The full-order residual
///   governs the answer, so R1's failure to generalise across contact positions
///   disqualifies the reduced *solve* and not a reduced *predictor*. That
///   asymmetry is why this candidate was worth the run.
/// - **A modal predictor must be used as a PROJECTION, never as a rescaled
///   shape.** Rescaling a mode to match the driven-node displacement costs
///   `24×` over not rescaling it, and no scalar correction tried recovers it.
/// - **The in-plane components are what hurt.** Adding a POD mode's `x`/`y`
///   field to an equivalent transverse profile costs `20×`; the `z` profile
///   itself is worth only `2.5×`. Why is not understood.
///
/// ⚠ `SIGMA_FLOOR_REL` in `reduced::pod` is not a floor for this use. It marks
/// where a mode is distinguishable from round-off, not where it is accurate,
/// and the gap is a square root — a basis fitted for a predictor should be
/// truncated at `√(ε/target)` on the relative spectrum instead.
///
/// ⇒ [`Self::Inertial`] is the best-measured choice on five of the six cells and
/// never failed on any — **but it is NOT the default, and deliberately so**: on a
/// stiff, quasi-static scene it is 3.5× WORSE than [`Self::PreviousState`]
/// (`block_sag`, 38 iterations against 11). Opt in per scene; do not assume it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum InitialGuess {
    /// `x⁰ = x_prev` — the previous step's converged position. The default,
    /// and the only variant that leaves the whole residual to Newton.
    #[default]
    PreviousState,
    /// `x⁰ = x̂ = x_prev + Δt·v_prev` — constant-velocity extrapolation.
    ///
    /// Zeroes the inertial term `(m/Δt²)·(x − x̂)` **on every free DOF**,
    /// exactly: the expression is the same fused
    /// `dt.mul_add(v_prev[i], x_prev[i])` the residual itself uses, so the
    /// cancellation is bit-exact rather than merely close. Leaves
    /// `f_int(x⁰) − f_ext`. (A CONSTRAINED DOF keeps `x_prev`, so its inertial
    /// term is *not* zeroed when `v_prev` is nonzero there — harmless, since
    /// constrained rows enter neither `free_residual_norm` nor the solve.)
    Inertial,
    /// `x⁰ = x̂ + Δt²·f_ext/m` — [`Self::Inertial`] plus the displacement the
    /// external load alone would produce.
    ///
    /// This is the **exact backward-Euler answer for a DOF with no internal
    /// force**, not the continuous constant-acceleration kinematic (which
    /// would carry a `½`): BE's own `x = x_prev + Δt·v`, `v = v_prev + Δt·a`
    /// composes to `x̂ + Δt²a` with no halving. It therefore zeroes every
    /// residual term except `f_int(x⁰)`, and is the "predictive position" the
    /// incremental-potential contact literature starts its solves from.
    ///
    /// **A zero lumped mass degrades this variant to [`Self::Inertial`] on that
    /// DOF rather than dividing.** Two distinct sources, and only one is
    /// impossible: an unreferenced vertex is auto-pinned at construction (see
    /// `construct`'s `effective_pinned` walk) so it never reaches a free DOF —
    /// but **`SolverConfig::density = 0.0` zeroes the mass of the whole model
    /// at once**, and that is a supported static-solve configuration, not a
    /// mistake (`slender_bending_matches_analytic`'s `STATIC_DENSITY`). With no
    /// mass there is no inertial term in the residual at all, so there is
    /// nothing for `Δt²·f_ext/m` to predict against.
    InertialWithLoad,
}

/// Solver configuration — integration parameters the skeleton scene
/// (spec §2) consumes. `skeleton()` returns the spec-§2 defaults.
///
/// # Construction
///
/// **Start from [`Self::skeleton`] and mutate the fields you need.** That is
/// how every call site in this workspace already builds one, and with
/// `#[non_exhaustive]` it is the only way an external crate can.
///
/// ```
/// use sim_soft::SolverConfig;
/// let mut cfg = SolverConfig::skeleton();
/// cfg.dt = 1.0 / 60.0;
/// cfg.gravity_z = -9.81;
/// ```
///
/// That fence is a REAL doctest, deliberately. A doctest compiles as an
/// external crate, which is exactly the vantage point `#[non_exhaustive]`
/// changes — so it is a live check that the only construction route this type
/// leaves outsiders still works. An `ignore` fence here would be unverified
/// documentation of the very contract the attribute exists to enforce.
///
/// `#[non_exhaustive]` is deliberate and was added when `initial_guess`
/// landed. This struct is a bag of independent integration knobs that grows
/// as the solver does — `gravity_z`, `lm_regularization`, `friction_mu`,
/// `fbar` and `initial_guess` were all added after v1.0.0 — and with public
/// fields and no marker, **every one of those additions is a semver break for
/// any external struct literal.** Closing it converts an open-ended series of
/// breaks into exactly one, taken here, at the cost of a construction idiom
/// the workspace had already adopted unanimously. Adding a field is now a
/// minor-version change; only removing or retyping one is major.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
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
    /// ⚠ **Qualitative / stability cure, NOT quantitatively accurate.** It lets
    /// `ν = 0.49` *converge* (where plain Tet4 locks/stalls) with the right
    /// deformation *shape*, but over-softens the near-incompressible response
    /// ~5 % at ν=0.4 growing to ~21 % at ν=0.49 (mesh-converged; measured vs the
    /// analytic Lamé oracle — the nodal-patch average over-relaxes). Use it for
    /// ν=0.49 stability and qualitative/relative work; the quantitatively
    /// accurate path is higher-order (Tet10) — see the module docs and
    /// `docs/SIM_SOFT_TET10_PLAN.md`.
    ///
    /// PR1 is FORWARD-ONLY: the differentiable paths (`step`, the VJP /
    /// sensitivity methods) **panic** when `fbar` is set rather than silently
    /// return a tangent that omits the F-bar neighbor coupling. Use
    /// `replay_step` for forward-only F-bar; the differentiability leaf (PR2)
    /// wires the coupling into the adjoint. (Mirrors the `friction_mu`
    /// forward-only-in-PR1 contract above.)
    pub fbar: bool,
    /// Where Newton starts each step. Default [`InitialGuess::PreviousState`]
    /// = start from `x_prev`, **bit-equal** to the pre-predictor path (the
    /// `PreviousState` arm of `apply_initial_guess` is a no-op, the same
    /// short-circuit shape as `gravity_z = 0.0` and `fbar = false`).
    ///
    /// The non-default variants trade a cheaper Newton path for a first
    /// iterate that may sit outside the convergence basin — see
    /// [`InitialGuess`] for the risk and for what each variant zeroes.
    ///
    /// **Honoured by the reduced solver too, except for one variant.**
    /// `backward_euler::reduced`'s `ReducedNewtonSolver` borrows the full solver
    /// (`&CpuNewtonSolver`) and reads this same config, so the two cannot be
    /// configured apart — which is why it implements the guess rather than
    /// ignoring it. [`InitialGuess::Inertial`] reproduces the oracle's `x̂`
    /// EXACTLY in reduced coordinates (`x_rest + Φ(q + Δt·q̇) = x_prev + Δt·v`),
    /// for either `Inner`.
    ///
    /// ⚠ [`InitialGuess::InertialWithLoad`] is **refused with a panic** there:
    /// its reduced form wants `(ΦᵀMΦ)⁻¹Φᵀf_ext`, free only under `Inner::Mass`,
    /// and the variant is killed on the evidence anyway (recon §2f). The refusal
    /// is a panic, not a [`SolverFailure`](crate::SolverFailure) — a `try_`-style
    /// caller cannot recover — so it is a configuration error caught at
    /// `ReducedNewtonSolver::new`, not a runtime condition.
    pub initial_guess: InitialGuess,
}

impl SolverConfig {
    /// Scope §2 defaults for the walking-skeleton scene: `dt = 1e-2`,
    /// `tol = 1e-10` (five digits below gradcheck's 1e-5 bar),
    /// `density = 1030` (silicone-class), up to 10 Newton iterations +
    /// 20 backtracks per iteration. `gravity_z = 0` — the
    /// drop-and-rest fixture opts in by mutating the field on a
    /// constructed config (mirrors the Hertzian and compressive-block
    /// fixtures' `STATIC_DT` bumping pattern).
    /// `initial_guess = PreviousState` — Newton starts from `x_prev`, the
    /// pre-predictor behavior; a caller opts into extrapolation the same way.
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
            initial_guess: InitialGuess::PreviousState,
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
