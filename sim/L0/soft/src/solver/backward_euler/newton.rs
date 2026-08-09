//! Newton outer loop + Armijo line-search for
//! [`CpuNewtonSolver`](super::CpuNewtonSolver).

use faer::sparse::Triplet;
use sim_ml_chassis::Tensor;

use crate::contact::{ActivePairsFor, ContactModel};
use crate::element::Element;
use crate::material::{InversionHandling, Material};
use crate::mesh::{Mesh, TetId};
use crate::solver::lm::LmState;
use crate::solver::{CpuTape, NewtonStep, SolverFailure};

use super::helpers::{
    armijo_stall_panic_message, deformation_gradient, element_node_ids, extract_element_dof_values,
    residual_into,
};
use super::{CpuNewtonSolver, GaussGeometry};

/// Armijo sufficient-decrease constant (scope §5 R-1).
const ARMIJO_C1: f64 = 1e-4;

/// Local stall-info carrier returned by [`CpuNewtonSolver::armijo_backtrack`]
/// when the line search exhausts its backtrack budget (F3.3 per
/// `docs/F3_LM_REGULARIZATION_SPEC.md` §2.5).
///
/// Distinct from the public [`SolverFailure::ArmijoStall`] variant —
/// shape mirrors the spec's local-struct fields exactly. The pre-F3
/// panic message included the per-call `final_alpha`; spec drops it
/// from this struct (and so the panic-message text in
/// `solve_impl`'s translation arm loses the `final α 4.77e-7`
/// substring vs pre-F3). No regression-test pins on that substring;
/// the substantive panic surface (panic-on-stall behavior + Newton
/// `iter` + `r_norm`) is preserved.
#[derive(Debug)]
pub(super) struct ArmijoStallInfo {
    pub(super) x_curr: Vec<f64>,
    pub(super) iter: usize,
    pub(super) r_norm: f64,
}

impl<E, Msh, C, M, const N: usize, const G: usize> CpuNewtonSolver<E, Msh, C, M, N, G>
where
    E: Element<N, G>,
    Msh: Mesh<M>,
    M: Material,
    C: ContactModel + ActivePairsFor<M>,
{
    /// Check every per-tet [`Material::validity`] domain against the
    /// deformation gradient `F` evaluated at `x_curr` (Phase 4 commit
    /// 12, IV-7 per scope memo Decision Q).
    ///
    /// First-violator-wins: walks tets in ascending `tet_id` order, evaluates
    /// `F = Σ_a x_{a,i} · ∂N_a/∂X_j`, and returns
    /// [`SolverFailure::ValidityViolation`] for the first tet whose `F` falls
    /// outside the declared [`crate::ValidityDomain`] (the `step` /
    /// `replay_step` wrappers turn that `Err` into the panic; this method
    /// itself does not panic on a violation). The two slots checkable from `F`
    /// for every base [`Material`] impl Phase 4 ships are
    /// `inversion` (`det F ≤ 0`, or non-finite, under
    /// [`InversionHandling::RequireOrientation`]) and
    /// `max_stretch_deviation` (max `|σ_i − 1|` over the three
    /// singular values `σ_i` of `F`).
    ///
    /// **The two slots read different `F`s, deliberately.** `inversion` is
    /// swept over the per-Gauss-point [`super::GaussGeometry`] — the points
    /// where [`Material::first_piola`] is actually evaluated — because a
    /// `det F ≤ 0` the gate does not see becomes a silent `NaN` downstream
    /// rather than a caught violation. `max_stretch_deviation` still reads
    /// the single-point corner block. Moving it too is a separate, wider
    /// change and is deliberately not bundled here: unlike `inversion` — whose
    /// corner-block reading is not a quantity anything evaluates — the corner
    /// stretch is a real bound that existing Tet10 consumers currently satisfy,
    /// and the two available shapes differ. *Adding* per-Gauss-point bounds can
    /// only shrink the valid set; *replacing* the corner check with them is not
    /// monotone in either direction, since the corner `F` is a function of the
    /// four corner nodes alone and is not any combination of the Gauss-point
    /// `F`s. Which shape is right wants its own measured pass. For
    /// Tet4 the distinction is vacuous: `G == 1` and the centroid gradient is
    /// bit-identical to the corner block, so both slots see one `F`.
    ///
    /// The other four
    /// [`crate::ValidityDomain`] slots are construction-time
    /// (`poisson_range`) or decorator-only (`temperature_range`,
    /// `strain_rate_range`, `max_rotation` infinite for the
    /// scalar-isotropic NH baseline) and not checked here.
    ///
    /// Diagnostic-only at the solver level — Decision K's "Newton
    /// hot path does not branch on diagnostic metadata" framing
    /// applies to the interface flag, not to validity; this check
    /// runs at two step boundaries per [`Solver::step`](crate::solver::Solver::step) call:
    /// (1) before the Newton loop starts (the original Decision Q
    /// "at step start" framing — catches invalid warm-starts), AND
    /// (2) at end of solve before returning a converged result
    /// (catches Newton converging to an invalid equilibrium —
    /// without this check, an invalid converged state silently
    /// flows to the next step's start check, making the failure
    /// step-delayed + the failed step's recorded output
    /// physically meaningless).  Both boundaries fail closed on the
    /// first violation rather than degrading silently.
    ///
    /// (⚠ boundary (2)'s `inversion` slot is nearly dead — see
    /// [`Self::check_orientation_at_gauss_points`].)  See
    /// `docs/archive/CANDIDATE_E_B_FALSIFICATION_BOOKMARK.md` §10 for the
    /// motivating finding (cavity > 5 mm sliding-ramp step 1
    /// converged to `σ_max = 2.05` at tet 3206, was only caught at
    /// step 2's start check pre-this-fix).  The book Part 2 §00
    /// §02 prescription is a runtime warning; Decision Q upgrades
    /// to fail-closed semantics for Phase 4.
    ///
    /// # Errors
    ///
    /// Returns [`SolverFailure::ValidityViolation`] on the first violator, carrying the violated
    /// `tet_id` and the structured `message`
    /// `"validity violation at tet {id}: {slot} = {value:.3} ..."` (where `{slot}` is one of
    /// `max_stretch_deviation` / `max_principal_stretch` / `min_principal_stretch` / `inversion`).
    /// The `try_step`/`try_replay_step` callers surface this `Err` so a feasibility-aware caller can
    /// skip the design; the panic-path `step`/`replay_step` re-`panic!` with the same `message`, so
    /// the IV-7 `#[should_panic(expected = "max_stretch_deviation")]` slot-substring contract holds.
    //
    // similar_names: `tet_id`/`tet` mirrors the assembly methods.
    // cast_possible_truncation: same Mesh-trait API tax as the
    // assembly methods.
    #[allow(clippy::similar_names, clippy::cast_possible_truncation)]
    fn check_validity_at_step_start(&self, x_curr: &[f64]) -> Result<(), SolverFailure> {
        debug_assert!(x_curr.len() == self.n_dof);
        let materials = self.mesh.materials();
        // `zip` truncates to the shorter iterator, and a gate that silently skips
        // tets fails open. Hard `assert!` so that holds in release too.
        assert_eq!(
            self.element_geometries.len(),
            self.gauss_geometries.len(),
            "validity gate would silently skip tets if the two geometry caches diverged"
        );
        for (tet_id, (geom, gauss_geom)) in self
            .element_geometries
            .iter()
            .zip(&self.gauss_geometries)
            .enumerate()
        {
            let validity = materials[tet_id].validity();

            // Inversion check first, for two reasons — neither of which is that
            // the stretch verdict would be wrong otherwise. `|σ − 1|` is a
            // genuine bound whatever the orientation, since singular values are
            // orientation-blind (`σ(F) = σ(RF)` for a reflection `R`).
            //   1. Attribution: first-violator-wins, so an inverted element
            //      should be reported on the slot that names the actual defect
            //      rather than on whatever stretch bound it also happens to trip.
            //   2. The stretch reductions below SWALLOW non-finite values (see
            //      their comment), so running the finiteness-checking slot first
            //      is what keeps a `NaN` state from passing silently.
            // Programs that allow `det F <= 0` declare a non-`RequireOrientation`
            // inversion handler, and Phase 4 has none — Phase H may add
            // `Barrier` / `OptIn` variants when an impl needs them. ⚠ A variant
            // that skips this sweep arms the NaN hole named in (2).
            if matches!(validity.inversion, InversionHandling::RequireOrientation) {
                self.check_orientation_at_gauss_points(x_curr, tet_id, gauss_geom)?;
            }

            // Principal-stretch bounds: SVD `F = U Σ V^T` gives
            // singular values `σ_i` which are the principal stretches.
            // `f.svd_unordered(false, false)` skips U/V (we only need
            // σ); cheap O(27) FLOPs per tet.
            //
            // ⚠ These reductions SWALLOW non-finite values: Rust's `f64::max` /
            // `f64::min` return the non-NaN operand, so a `NaN` σ would report
            // `max_dev = 0.0` / `min_sigma = INFINITY` and PASS. Safe only
            // because the inversion sweep above rejects a non-finite `det F`
            // first, for every material that declares `RequireOrientation` —
            // which today is all of them. Adding a variant that skips the sweep
            // means adding an `is_finite` guard here.
            //
            // ⚠ Which orientations reach here differs by element. For Tet4 the
            // sweep above IS this corner block (single Gauss pair), so nothing
            // orientation-negative gets this far. For Tet10 it is not: the sweep
            // reads the Gauss points, so a corner block that is itself
            // orientation-negative is no longer gated at all — a state that was
            // `Err` before this slot moved and is now `Ok`. Deliberate: no
            // constitutive law is ever evaluated at the Tet10 corner `F` (only
            // this bound is), so its orientation is not a property of the state
            // that the physics can observe. Harmless to this slot too, whose σ
            // are orientation-blind either way.
            //
            // Two gate flavors (Yeoh arc memo D8): if either of the new
            // asymmetric bounds is `Some`, gate per-bound; else fall
            // back to the legacy NH symmetric `max_i |σ_i - 1|` bound.
            //
            // Edge case: `(Some, None)` or `(None, Some)` checks only
            // the populated bound — the other direction is unchecked.
            // No production constructor reaches that state today (every
            // `SiliconeMaterial::to_yeoh` sets both via
            // `with_principal_stretch_bounds`); future asymmetric-only
            // callers opt into the unchecked direction by construction.
            //
            // Still read from the single-point corner geometry (for Tet4 the
            // constant strain; for Tet10 the affine corner block) — the
            // inversion sweep above deliberately does not share this `F`. See
            // this method's doc for why only one of the two slots moved.
            let verts = self.mesh.tet_vertices(tet_id as TetId);
            let x_elem = extract_element_dof_values(x_curr, &verts);
            let f = deformation_gradient(&x_elem, &geom.grad_x_n);
            let svd = f.svd_unordered(false, false);
            let sigma = svd.singular_values;
            match (
                validity.max_principal_stretch,
                validity.min_principal_stretch,
            ) {
                (None, None) => {
                    let max_dev = sigma
                        .iter()
                        .map(|s| (s - 1.0).abs())
                        .fold(0.0_f64, f64::max);
                    let bound = validity.max_stretch_deviation;
                    if max_dev > bound {
                        return Err(SolverFailure::ValidityViolation {
                            tet_id,
                            message: format!(
                                "validity violation at tet {tet_id}: max_stretch_deviation \
                                 = {max_dev:.3} exceeds bound {bound:.3} (singular values \
                                 of F = [{s0:.3}, {s1:.3}, {s2:.3}]). Phase 4 scope memo \
                                 Decision Q fail-closed semantics.",
                                s0 = sigma[0],
                                s1 = sigma[1],
                                s2 = sigma[2],
                            ),
                        });
                    }
                }
                (max_p, min_p) => {
                    if let Some(max) = max_p {
                        let max_sigma = sigma.iter().fold(0.0_f64, |a, &b| a.max(b));
                        if max_sigma > max {
                            return Err(SolverFailure::ValidityViolation {
                                tet_id,
                                message: format!(
                                    "validity violation at tet {tet_id}: max_principal_stretch \
                                     = {max_sigma:.3} exceeds bound {max:.3} (singular values \
                                     of F = [{s0:.3}, {s1:.3}, {s2:.3}]). Phase 4 scope memo \
                                     Decision Q fail-closed semantics.",
                                    s0 = sigma[0],
                                    s1 = sigma[1],
                                    s2 = sigma[2],
                                ),
                            });
                        }
                    }
                    if let Some(min) = min_p {
                        let min_sigma = sigma.iter().fold(f64::INFINITY, |a, &b| a.min(b));
                        if min_sigma < min {
                            return Err(SolverFailure::ValidityViolation {
                                tet_id,
                                message: format!(
                                    "validity violation at tet {tet_id}: min_principal_stretch \
                                     = {min_sigma:.3} below bound {min:.3} (singular values \
                                     of F = [{s0:.3}, {s1:.3}, {s2:.3}]). Phase 4 scope memo \
                                     Decision Q fail-closed semantics.",
                                    s0 = sigma[0],
                                    s1 = sigma[1],
                                    s2 = sigma[2],
                                ),
                            });
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// The `inversion` slot for one element: `det F > 0` at **every Gauss point**.
    ///
    /// Swept over [`GaussGeometry`], not the corner block, because orientation has to
    /// hold wherever the constitutive model is actually evaluated — these are the same
    /// per-point `grad_x_n` that `assemble_global_int_force`'s elastic branch builds each
    /// `F` from before handing it to [`Material::first_piola`].
    ///
    /// The F-bar branch of that assembly is the one exception, and it is covered rather
    /// than missed: it evaluates a patch-modified `F*` off the corner block, but it is
    /// Tet4-only (asserted in `try_solve_impl`, above both of this gate's call sites), and
    /// for Tet4 the corner block and the single Gauss pair are the same tensor. Its
    /// `det F* = J̄` is a positive-weighted average of exactly the per-element `J_e` this
    /// sweep checks, so `J̄ > 0` follows from the sweep passing.
    ///
    /// For **Tet4**, on a finite `det F`, this is verdict-identical to the pre-sweep
    /// corner check (`G == 1`, the centroid point, bit-identical to
    /// `ElementGeometry::grad_x_n`) — pinned by
    /// `tet4_inversion_verdict_survives_the_gauss_sweep`, whose cases are ones the
    /// corner check rejected too. The `is_finite` clause is the one Tet4 verdict that
    /// *did* change: a non-finite `det F` used to pass the bare `<= 0.0` predicate
    /// (every comparison against `NaN` is false). New for both element types, pinned by
    /// `non_finite_state_is_rejected_by_the_inversion_gate`.
    ///
    /// For **Tet10** the two `F`s differ outright: the corner block is an affine proxy
    /// that cannot see a midside-driven inversion at an interior Gauss point, and such an
    /// inversion does not stay missed — `first_piola` takes `ln(det F)` of a negative
    /// determinant and returns `NaN`, which reaches Newton as a `NaN` residual and
    /// surfaces much later as a misattributed "non-SPD tangent" Armijo stall.
    ///
    /// ## The three verdict changes, stated plainly
    ///
    /// 1. **`Err` → `Err`, re-attributed** (the common case): a Tet10 Gauss-point
    ///    inversion that used to surface as an `ArmijoStall` blaming the tangent now
    ///    names the element and the point.
    /// 2. **`Ok` → `Err`**: an element with **no free DOFs**. `free_residual_norm` reads
    ///    only `free_dof_indices`, so a `NaN` confined to pinned entries was never
    ///    observed and such a solve converged with an inverted element in it. Pinned by
    ///    `fully_pinned_inverted_element_no_longer_converges`.
    /// 3. **`Err` → `Ok`**: a Tet10 whose *corner block* is orientation-negative while
    ///    every Gauss point stays positive. The corner block is no longer gated at all
    ///    (see the stretch check's comment for why that is deliberate). No fixture in
    ///    the crate produces such a state and none is added here.
    ///
    /// Boundary (2) of the two the caller runs — the converged-state check — is nearly
    /// dead for this slot: reaching it requires `r_norm < tol`, which a `NaN`-poisoned
    /// `f_int` cannot satisfy, so only case 2 above can trip it there.
    ///
    /// [`CpuNewtonSolver::min_gauss_det_ratio`] reports the same quantity this gate
    /// tests (`det J_def(ξ_q) / det J_rest(ξ_q)` *is* `det F` at that point), by a
    /// separate route that reads raw node coordinates rather than the cached
    /// `grad_x_n`. Keep the two consistent.
    ///
    /// # Errors
    ///
    /// Returns [`SolverFailure::ValidityViolation`] naming the tet and the 0-based Gauss
    /// index of the first non-positive (or non-finite) `det F`.
    //
    // cast_possible_truncation: the Mesh-trait API tax, as in the assembly methods.
    #[allow(clippy::cast_possible_truncation)]
    fn check_orientation_at_gauss_points(
        &self,
        x_curr: &[f64],
        tet_id: usize,
        gauss_geom: &GaussGeometry<N, G>,
    ) -> Result<(), SolverFailure> {
        let nodes = element_node_ids::<M, Msh, N>(&self.mesh, tet_id as TetId);
        let x_nodes = extract_element_dof_values(x_curr, &nodes);
        for (q, (grad_x_n, _)) in gauss_geom.gauss.iter().enumerate() {
            let det_f = deformation_gradient(&x_nodes, grad_x_n).determinant();
            // `is_finite` as well as `<= 0.0`, because every float comparison
            // against a NaN is false: the bare `<=` form lets a non-finite
            // `det F` through, and a non-finite state is exactly what this gate
            // exists to stop from reaching `first_piola`.
            if !det_f.is_finite() || det_f <= 0.0 {
                return Err(SolverFailure::ValidityViolation {
                    tet_id,
                    message: format!(
                        "validity violation at tet {tet_id}: inversion = det F = \
                         {det_f:.3} at Gauss point {q} (0-based) of {G} violates \
                         RequireOrientation handler (must be finite and strictly \
                         positive). Phase 4 scope memo Decision Q fail-closed \
                         semantics."
                    ),
                });
            }
        }
        Ok(())
    }

    /// Free-DOF residual norm (scope §5 R-1 convergence criterion).
    /// Reads only the entries at `self.free_dof_indices`; pinned-DOF
    /// residual entries are excluded from the convergence test.
    fn free_residual_norm(&self, r_full: &[f64]) -> f64 {
        debug_assert!(r_full.len() == self.n_dof);
        self.free_dof_indices
            .iter()
            .map(|&idx| r_full[idx] * r_full[idx])
            .sum::<f64>()
            .sqrt()
    }

    /// Inner solver: pure-function-of-θ Newton loop. Shared by `step`
    /// and `replay_step`.
    ///
    /// F3.3: thin panic-on-failure wrapper around [`Self::try_solve_impl`].
    /// All three F3 failure surfaces (`SolverFailure::ArmijoStall` /
    /// `NewtonIterCap` / `DoublyFailedFactor`) panic here with pre-F3
    /// messages preserved bit-equal — the source-level body differs
    /// from pre-F3 but observable behavior at this method's surface
    /// is unchanged. Graceful-failure consumers use `try_step` /
    /// `try_replay_step` which route through `try_solve_impl`
    /// directly.
    ///
    /// # Panics
    /// - Newton exceeds `config.max_newton_iter` iterations without
    ///   reaching `config.tol`.
    /// - Armijo backtracks exceed `config.max_line_search_backtracks`.
    /// - Tangent factorization fails doubly (Llt tripped non-PD AND
    ///   the A2 Lu fallback also failed — see
    ///   `Self::factor_free_tangent`). This is a model-level
    ///   degeneracy, not runtime-recoverable.
    /// - A tet leaves the material's validity domain (Decision Q —
    ///   over-stretch / inversion), at step start or at the converged
    ///   state. (`try_step`/`try_replay_step` return this as
    ///   `SolverFailure::ValidityViolation` instead.)
    //
    // panic: scope §3 R-1 (3-5-iter convergence prediction) cap +
    // Armijo cap are book-level findings, not runtime-recoverable
    // conditions at the `step()` API surface. F3.3 widens the
    // recoverable surface via `try_step` without changing `step`'s
    // panic contract.
    #[allow(clippy::panic)]
    pub(super) fn solve_impl(
        &self,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta: &Tensor<f64>,
        dt: f64,
    ) -> (NewtonStep<CpuTape>, f64) {
        self.try_solve_impl(x_prev, v_prev, theta, dt)
            .unwrap_or_else(|fail| match fail {
                SolverFailure::ArmijoStall {
                    last_iter,
                    last_r_norm,
                    ..
                } => panic!("{}", armijo_stall_panic_message(last_iter, last_r_norm)),
                SolverFailure::NewtonIterCap { max_iter, .. } => panic!(
                    "Newton failed to converge within {max_iter} iterations at \
                     tol {tol:e}. Likely causes: θ drives system out of R-2's \
                     SPD region, or spec §3 R-1's assumption of 3-5 iter \
                     convergence from zero initial guess is wrong for this θ.",
                    tol = self.config.tol,
                ),
                SolverFailure::DoublyFailedFactor { context, .. } => panic!("{context}"),
                // Decision Q fail-closed: `step`/`replay_step` re-panic with the verbatim
                // validity message (the `try_` path returns it as `Err` instead).
                SolverFailure::ValidityViolation { message, .. } => panic!("{message}"),
            })
    }

    /// F3 recon candidate A — gated factor + solve + Armijo for one
    /// Newton iter (per `docs/F3_RECON_A_GATED_LM_SPEC.md` §2.2 + §2.3).
    /// Used exclusively by [`Self::try_solve_impl`].
    ///
    /// **First pass** (always): factor + solve with `LmState::disabled`
    /// → `δ_LU`; [`Self::armijo_backtrack`] on `δ_LU`. Bit-equal to
    /// pre-F3 LU + Armijo.
    ///
    /// **First pass succeeds** → return the Armijo-accepted iterate.
    ///
    /// **First pass fails AND outer `lm_state` is disabled** (§2.3
    /// short-circuit): return the first-pass failure directly. NO
    /// second factor + solve attempted — preserves the F3 spec's §F3.1
    /// bit-equal-when-dormant contract (no 2× factor wall-clock cost
    /// at LM-disabled stalls).
    ///
    /// **First pass fails AND outer `lm_state` is active** (§2.2
    /// ESCALATION): re-factor + re-solve using the outer persistent
    /// `lm_state` (which carries cross-Newton-iter λ per F3 spec §2.2).
    /// The first non-PD detection at this iter seeds-or-bumps λ via the
    /// inner `factor_free_tangent` retry loop. The LM-rescued `δ_LM`
    /// goes through `armijo_backtrack`; if Armijo accepts → return; if
    /// the LM step ALSO Armijo-stalls → return
    /// `Err(SolverFailure::ArmijoStall)` (no further escalation —
    /// already in the LM-rescue regime per §2.2 step 4b.ii).
    ///
    /// First-pass failure info (`DoublyFailedFactor` context,
    /// `ArmijoStall` `r_norm`) is DISCARDED on escalation per spec
    /// §2.5 ("the LU step already had its chance; escalation is the
    /// recovery attempt; if escalation also fails, THAT's the
    /// committed-iterate failure for the surfaced `SolverFailure`
    /// variant").
    //
    // too_many_arguments: 10 inputs mirror the residual + Newton-iter
    // formula's reads; bundling into a struct adds name-the-fields
    // ceremony for a sole caller (try_solve_impl) with no readability
    // gain (try_solve_impl already names its locals identically).
    #[allow(clippy::too_many_arguments)]
    fn try_gated_factor_solve_armijo(
        &self,
        triplets: &[Triplet<usize, usize, f64>],
        r_full: &[f64],
        x_curr: &[f64],
        x_prev_vec: &[f64],
        v_prev_vec: &[f64],
        f_ext: &[f64],
        dt: f64,
        r_norm: f64,
        newton_iter: usize,
        lm_state: &mut LmState,
    ) -> Result<Vec<f64>, SolverFailure> {
        // First pass: LM-disabled (bit-equal to pre-F3).
        let mut lm_disabled = LmState::disabled();
        let first_pass_outcome = self
            .try_factor_and_solve_free(triplets, r_full, newton_iter, r_norm, &mut lm_disabled)
            .map_err(|info| SolverFailure::DoublyFailedFactor {
                x_partial: x_curr.to_vec(),
                last_iter: newton_iter,
                context: info.context,
            })
            .and_then(|delta_lu| {
                self.armijo_backtrack(
                    x_curr,
                    x_prev_vec,
                    v_prev_vec,
                    f_ext,
                    dt,
                    &delta_lu,
                    r_norm,
                    newton_iter,
                )
                .map_err(|stall| SolverFailure::ArmijoStall {
                    x_partial: stall.x_curr,
                    last_iter: stall.iter,
                    last_r_norm: stall.r_norm,
                })
            });

        match first_pass_outcome {
            Ok(x_accepted) => Ok(x_accepted),
            Err(failure) if !lm_state.is_active() => {
                // §2.3 short-circuit: no LM rescue mechanism available.
                Err(failure)
            }
            Err(_first_pass_failure_discarded) => {
                // §2.2 ESCALATION via the OUTER persistent lm_state.
                let delta_lm = self
                    .try_factor_and_solve_free(triplets, r_full, newton_iter, r_norm, lm_state)
                    .map_err(|info| SolverFailure::DoublyFailedFactor {
                        x_partial: x_curr.to_vec(),
                        last_iter: newton_iter,
                        context: info.context,
                    })?;
                self.armijo_backtrack(
                    x_curr,
                    x_prev_vec,
                    v_prev_vec,
                    f_ext,
                    dt,
                    &delta_lm,
                    r_norm,
                    newton_iter,
                )
                .map_err(|stall| SolverFailure::ArmijoStall {
                    x_partial: stall.x_curr,
                    last_iter: stall.iter,
                    last_r_norm: stall.r_norm,
                })
            }
        }
    }

    /// Graceful-failure counterpart to [`Self::solve_impl`] (F3.3 per
    /// `docs/F3_LM_REGULARIZATION_SPEC.md` §2.5). Same Newton loop,
    /// but returns `Result<(NewtonStep, f64), SolverFailure>` instead
    /// of panicking. The `f64` is the Newton-final λ (threaded into
    /// the IFT-adjoint factor by [`Self::try_step`] per spec §2.1).
    ///
    /// `x_partial` on each `SolverFailure` variant is `x_curr` at the
    /// START of the failed Newton iter (per spec §2.5 `ArmijoStall`
    /// docstring — committed-Newton-iterate semantics, NOT `trial_x`
    /// from Armijo backtracking, NOT a partial-Armijo-accepted `x`).
    /// `NewtonIterCap`'s `x_partial` is the most-recent
    /// armijo-accepted iterate (one full iter past the last failed
    /// convergence check).
    ///
    /// **F3 recon candidate A — gated LM activation** (per
    /// `docs/F3_RECON_A_GATED_LM_SPEC.md`). At each Newton iter, the
    /// inner factor + solve + Armijo is attempted TWICE in the failure
    /// path: first with LM SUPPRESSED (bit-equal to pre-F3 LU + Armijo
    /// — the cavity = 3 mm baseline preservation lever); on first-pass
    /// failure AND `lm_state.is_active()`, escalate to the persistent
    /// outer `lm_state`'s LM-rescued retry. At LM-disabled
    /// (`SolverConfig::lm_regularization == None`), the §2.3
    /// short-circuit returns the first-pass failure directly to
    /// preserve the F3 spec's §F3.1 bit-equal-when-dormant contract
    /// (no 2× factor cost at LM-disabled stalls). The OUTER
    /// `lm_state` carries cross-iter λ per F3 spec §2.2 persistence
    /// rule — only consumed by the escalation branch.
    pub(super) fn try_solve_impl(
        &self,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta: &Tensor<f64>,
        dt: f64,
    ) -> Result<(NewtonStep<CpuTape>, f64), SolverFailure> {
        assert!(
            x_prev.as_slice().len() == self.n_dof,
            "x_prev must have {} entries, got {}",
            self.n_dof,
            x_prev.as_slice().len(),
        );
        assert!(
            v_prev.as_slice().len() == self.n_dof,
            "v_prev must have {} entries, got {}",
            self.n_dof,
            v_prev.as_slice().len(),
        );
        assert!(dt > 0.0, "dt must be positive, got {dt}");
        // Belt-and-suspenders (Tet10 ladder rung 3a): F-bar is Tet4-only.
        // Its locking cure rests on the single-Gauss-point per-element
        // volumetric constraint (`fbar.rs`), which multi-Gauss-point Tet10
        // has no analog for. The differentiable path is already guarded in
        // `factor_at_position`; this guards the forward primal solve
        // (`solve_impl` and `try_step` both route here) so a forward-only
        // Tet10 solve with `fbar=true` can't slip through ungated. Live since
        // rung 3b lifted `new()`'s `N == 4` pin: a plain (`fbar=false`) Tet10
        // forward solve is now supported and passes this guard.
        assert!(
            !(self.config.fbar && N != 4),
            "F-bar is implemented for Tet4 (N=4) only; got N={N} with config.fbar=true. \
             F-bar's single-Gauss-point volumetric constraint has no multi-Gauss-point \
             Tet10 analog (see fbar.rs)."
        );

        let mut x_curr: Vec<f64> = x_prev.as_slice().to_vec();
        let x_prev_vec: Vec<f64> = x_prev.as_slice().to_vec();
        let v_prev_vec: Vec<f64> = v_prev.as_slice().to_vec();

        let mut f_ext = vec![0.0; self.n_dof];
        self.assemble_external_force(theta, &mut f_ext);

        // Decision Q validity check (step-start boundary): returns
        // `SolverFailure::ValidityViolation` on a tet leaving the
        // validity domain. `try_solve_impl` propagates it via `?` so
        // the `try_` path surfaces it as `Err` (a feasibility-aware
        // co-design caller skips the design); `solve_impl`/`step`
        // re-panic the verbatim Decision Q message (fail-closed
        // contract unchanged).
        self.check_validity_at_step_start(&x_curr)?;

        let mut f_int = vec![0.0; self.n_dof];
        let mut r_full = vec![0.0; self.n_dof];

        let mut lm_state = self
            .config
            .lm_regularization
            .map_or_else(LmState::disabled, LmState::from_config);

        let mut last_r_norm = 0.0_f64;

        for newton_iter in 0..self.config.max_newton_iter {
            self.assemble_global_int_force(&x_curr, &x_prev_vec, dt, &mut f_int);
            residual_into(
                &x_curr,
                &x_prev_vec,
                &v_prev_vec,
                &f_int,
                &f_ext,
                &self.mass_per_dof,
                dt,
                &mut r_full,
            );
            let r_norm = self.free_residual_norm(&r_full);
            last_r_norm = r_norm;

            if r_norm < self.config.tol {
                // Decision Q validity check at converged state — sister
                // of the step-start check at line 1525. Without this,
                // Newton can converge to a deformation field where some
                // tet's F violates max_stretch_deviation / inversion;
                // the failure surfaces only at the NEXT step's start
                // check, masking which step actually produced the
                // invalid state + leaving the failed step's recorded
                // output physically meaningless. See
                // `check_validity_at_step_start`'s docstring for the
                // motivating finding + the two-boundary check contract.
                self.check_validity_at_step_start(&x_curr)?;
                return Ok((
                    NewtonStep::new_converged(x_curr, newton_iter, r_norm),
                    lm_state.lambda(),
                ));
            }

            let triplets = self.assemble_free_hessian_triplets(&x_curr, Some(&x_prev_vec), dt);
            x_curr = self.try_gated_factor_solve_armijo(
                &triplets,
                &r_full,
                &x_curr,
                &x_prev_vec,
                &v_prev_vec,
                &f_ext,
                dt,
                r_norm,
                newton_iter,
                &mut lm_state,
            )?;
        }

        Err(SolverFailure::NewtonIterCap {
            x_partial: x_curr,
            max_iter: self.config.max_newton_iter,
            last_r_norm,
        })
    }

    /// Armijo backtracking per scope §5 R-1: shrink α geometrically
    /// until `‖r(x + α δ)‖ ≤ (1 - c₁ α) ‖r‖`. Updates only the free
    /// DOFs of `x` (looked up via `self.free_dof_indices`); pinned
    /// DOFs stay at their `x_curr` values.
    ///
    /// **A non-finite `trial_norm` is a rejected trial, not a fault.** This is
    /// why `Material::first_piola` returns `NaN` on an inverted `F` rather than
    /// panicking: a trial state may legitimately overshoot into inversion, the
    /// sufficient-decrease test below is false for `NaN` (every comparison
    /// against `NaN` is), so control reaches `alpha *= 0.5` and the next trial
    /// is rebuilt from `x_curr` — `assemble_global_int_force` re-zeroes `f_int`
    /// and `residual_into` rewrites every index, so the `NaN` cannot persist
    /// across trials. Panicking in the material would turn each such trial into
    /// an aborted solve.
    ///
    /// ⚠ What that does NOT claim is that the search always escapes. The loop
    /// is budgeted by `max_line_search_backtracks`, and even a non-inverted
    /// trial must still satisfy sufficient decrease, which needs a descent
    /// direction the tangent may not supply. A `NaN` trial can equally exhaust
    /// the budget and return [`ArmijoStallInfo`]. The claim is only that
    /// halving is reachable — so panicking would remove a usable outcome — not
    /// that recovery is guaranteed. Keeping a state out of inversion in the
    /// first place is `check_validity_at_step_start`'s job, at the boundaries
    /// where the state is a candidate equilibrium rather than a trial.
    ///
    /// F3.3: returns `Result<Vec<f64>, ArmijoStallInfo>` so callers
    /// can choose their own failure response. `solve_impl` (the
    /// existing panic-on-stall consumer) translates the local
    /// `ArmijoStallInfo` to a `panic!` with the pre-F3 message
    /// preserved bit-equal; `try_solve_impl` (the new
    /// graceful-failure consumer) maps it into the public
    /// [`SolverFailure::ArmijoStall`] variant.
    //
    // too_many_arguments: 8 inputs mirror the residual formula's
    // reads; bundling into a struct adds name-the-fields ceremony for
    // two callers with no readability gain.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn armijo_backtrack(
        &self,
        x_curr: &[f64],
        x_prev_vec: &[f64],
        v_prev_vec: &[f64],
        f_ext: &[f64],
        dt: f64,
        delta_free: &[f64],
        r_norm: f64,
        newton_iter: usize,
    ) -> Result<Vec<f64>, ArmijoStallInfo> {
        debug_assert!(x_curr.len() == self.n_dof);
        debug_assert!(delta_free.len() == self.n_free);
        let mut alpha = 1.0;
        // trial_x is initialized from x_curr each iteration via the free-DOF
        // re-write below; pre-allocate the buffer once and reuse across
        // backtracks.
        let mut trial_x: Vec<f64> = x_curr.to_vec();
        let mut trial_f_int = vec![0.0; self.n_dof];
        let mut trial_r = vec![0.0; self.n_dof];

        for _ in 0..=self.config.max_line_search_backtracks {
            // Reset free DOFs to `x_curr + α · δ`; pinned DOFs are
            // unchanged from the initial copy.
            for (free_idx, &full_idx) in self.free_dof_indices.iter().enumerate() {
                trial_x[full_idx] = x_curr[full_idx] + alpha * delta_free[free_idx];
            }
            self.assemble_global_int_force(&trial_x, x_prev_vec, dt, &mut trial_f_int);
            residual_into(
                &trial_x,
                x_prev_vec,
                v_prev_vec,
                &trial_f_int,
                f_ext,
                &self.mass_per_dof,
                dt,
                &mut trial_r,
            );
            let trial_norm = self.free_residual_norm(&trial_r);
            if trial_norm <= alpha.mul_add(-ARMIJO_C1, 1.0) * r_norm {
                return Ok(trial_x);
            }
            alpha *= 0.5;
        }
        Err(ArmijoStallInfo {
            x_curr: x_curr.to_vec(),
            iter: newton_iter,
            r_norm,
        })
    }

    // ── Cache-using assembly methods (live since Phase 2 commit 4b). ──
}
