//! Newton outer loop + Armijo line-search for
//! [`CpuNewtonSolver`](super::CpuNewtonSolver).

use faer::sparse::Triplet;
use nalgebra::SMatrix;
use sim_ml_chassis::Tensor;

use crate::Vec3;
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
use super::{CpuNewtonSolver, ElementGeometry, GaussGeometry};

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
    /// **The two slots read overlapping but different `F`s.** `inversion` is
    /// swept over the per-Gauss-point [`super::GaussGeometry`] — the points
    /// where [`Material::first_piola`] is actually evaluated, because a
    /// `det F ≤ 0` the gate does not see becomes a silent `NaN` downstream —
    /// AND over the single-point corner block, AND at the four reference
    /// corners for a higher-order element — the Stroud points are interior, so a
    /// corner-region fold is invisible to them, and the corner block sees only
    /// the four corner NODES, so a midside-driven fold is invisible to it too
    /// (measured at 0% caught on that class). The three stages catch disjoint
    /// things. `max_stretch_deviation` reads the corner block
    /// only. Moving it to the Gauss points too is a separate, wider
    /// change and is deliberately not bundled here: the corner
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
    /// (⚠ boundary (2) is NOT redundant for the `inversion` slot: F-bar hides
    /// an inversion from the `NaN` mechanism entirely, so that boundary is the
    /// only thing that catches it within the step — see
    /// [`Self::check_orientation`].)  See
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
    /// `"validity violation at tet {id}: {slot} = ..."` — where the value rendering
    /// is per-slot, not uniform: `inversion` emits `inversion = det F = {value:.3}`
    /// followed by one of three locators — `at Gauss point {q} (0-based) of {G}`,
    /// `on the corner block`, or `at reference corner {c} (0-based) of 4` — while
    /// the stretch slots emit `{slot} = {value:.3}`. `{slot}` is one of
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
            //      their comment), so SOME slot must reject a non-finite `det F`.
            //      ⚠ Order is not what secures that: both slots run in the same
            //      loop iteration, so a `NaN` is caught either way and only the
            //      ATTRIBUTION depends on which runs first. Reason (1) is the
            //      whole reason for the ordering; this is a reason the sweep must
            //      EXIST, which is a different claim.
            // Programs that allow `det F <= 0` declare a non-`RequireOrientation`
            // inversion handler, and Phase 4 has none — Phase H may add
            // `Barrier` / `OptIn` variants when an impl needs them. ⚠ A variant
            // that skips this sweep arms the NaN hole named in (2).
            if matches!(validity.inversion, InversionHandling::RequireOrientation) {
                self.check_orientation(x_curr, tet_id, geom, gauss_geom)?;
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
            // Nothing orientation-negative reaches here, for either element
            // type: the sweep above checks the Gauss points AND the corner
            // block, so both `F`s this slot could read are already known
            // positive-oriented.
            //
            // ⚠ An earlier revision of this comment said the opposite — that a
            // corner-negative Tet10 was "no longer gated at all", and called
            // that deliberate on the grounds that no constitutive law is
            // evaluated at the corner `F`. That was the rationalization for a
            // real hole: such an element passed the whole gate (this slot
            // abstains too, since σ are orientation-blind, so `diag(1,1,-1)`
            // reports `max_dev = 0`) and the solve returned `Ok` with a folded
            // element in it. The corner block is gated again. Do not restore
            // the argument.
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

    /// The `inversion` slot for one element: `det F > 0` at **every Gauss point**,
    /// on the element's **corner block**, and at its four **reference corners**
    /// (higher-order elements only).
    ///
    /// Swept over [`GaussGeometry`] — not only the corner block — because orientation has to
    /// hold wherever the constitutive model is actually evaluated — these are the same
    /// per-point `grad_x_n` that `assemble_global_int_force`'s elastic branch builds each
    /// `F` from before handing it to [`Material::first_piola`].
    ///
    /// The F-bar branch of that assembly is the one exception, and it is covered rather
    /// than missed: it evaluates a patch-modified `F*` off the corner block, but it is
    /// Tet4-only (asserted in `try_solve_impl`, above both of this gate's call sites), and
    /// for Tet4 the corner block and the single Gauss pair are the same tensor. Its
    /// `det F* = J̄` is a positive-weighted average of the per-element `J_e` this sweep
    /// checks, so `J̄ > 0` follows from the sweep passing. (One degenerate exception:
    /// `fbar::element_j_bar` seeds a zero-volume node with a hard-coded `1.0` rather
    /// than any `J_e`, so "exactly" would overstate it — the positivity conclusion is
    /// unaffected, since `1.0 > 0`.)
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
    ///
    /// 3. **`Ok` → `Err`, again**: a higher-order element folded at a reference
    ///    CORNER while its Gauss points and corner block all read positive. On a
    ///    randomized midside population this is 60% of the states that hide a
    ///    negative `det F` (see [`Self::check_orientation`]'s measured table), so
    ///    it is the largest of the three by volume. Pinned by
    ///    `a_fold_at_a_reference_corner_is_caught_when_gauss_and_corner_block_accept`.
    ///
    /// There is no `Err` → `Ok` class. An earlier revision of this change swept the Gauss
    /// points *instead of* the corner block, which introduced an `Err` → `Ok` class:
    /// a Tet10 whose corner block is orientation-negative while every Gauss point
    /// stays positive. That revision documented the class as unreachable — "no
    /// fixture in the crate produces such a state". It is reachable in about two
    /// element widths (mirror the corners through a plane, leave the midsides within
    /// a few element lengths — measured on the fixture: max coordinate 2.5 edge
    /// lengths, largest midside displacement 4.1), the stretch slot abstains on it because singular values are
    /// orientation-blind, and the solve returned `Ok` with a fully folded element in
    /// the mesh. This gate now checks both, so the class is closed rather than
    /// documented — see `corner_inverted_tet10_is_rejected_even_when_every_gauss_point_is_positive`.
    ///
    /// ⚠ Boundary (2) of the two the caller runs — the converged-state check — is
    /// **not** redundant, and an earlier revision of this doc said it was "nearly
    /// dead" on the reasoning that a `NaN`-poisoned `f_int` cannot satisfy
    /// `r_norm < tol`. That reasoning fails for every F-bar solve: with `J_e < 0` and
    /// a healthy patch, `theta = (j_bar / j).cbrt()` is NEGATIVE, so
    /// `det(theta * F) = theta^3 * J = J_bar > 0`, `first_piola` receives a positive
    /// determinant, `ln` stays finite and no `NaN` is ever produced. F-bar hides the
    /// inversion from the `NaN` mechanism entirely, so the converged-state check is
    /// the only thing that catches that class WITHIN the step. Deleting it would not
    /// lose the state outright — it flows to the next step's start check, as this
    /// method's own summary notes — but the failure becomes step-delayed and
    /// misattributed, and on the final step of a sequence there is no next check.
    /// ⚠ Argued, not gated: no fixture here runs an F-bar solve to an inverted
    /// converged state.
    ///
    /// [`CpuNewtonSolver::min_gauss_det_ratio`] reports the same quantity this gate
    /// tests (`det J_def(ξ_q) / det J_rest(ξ_q)` *is* `det F` at that point), by a
    /// separate route that reads raw node coordinates rather than the cached
    /// `grad_x_n`. Keep the two consistent.
    ///
    /// # Errors
    ///
    /// Returns [`SolverFailure::ValidityViolation`] naming the tet and, for a Gauss-point
    /// violation, the 0-based index of the first non-positive (or non-finite) `det F`.
    /// A corner-block violation carries no Gauss index — it is not located at one — and
    /// says `on the corner block` instead. A reference-corner violation says
    /// `at reference corner {c} (0-based) of 4`.
    //
    // cast_possible_truncation: the Mesh-trait API tax, as in the assembly methods.
    #[allow(clippy::cast_possible_truncation)]
    fn check_orientation(
        &self,
        x_curr: &[f64],
        tet_id: usize,
        geom: &ElementGeometry,
        gauss_geom: &GaussGeometry<N, G>,
    ) -> Result<(), SolverFailure> {
        let nodes = element_node_ids::<M, Msh, N>(&self.mesh, tet_id as TetId);
        let x_nodes = extract_element_dof_values(x_curr, &nodes);

        // (a) Every Gauss point — where the constitutive model is evaluated.
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

        // (b) The corner block — the element's own affine orientation.
        //
        // Sweeping the Gauss points is what this change is FOR, but sweeping
        // them INSTEAD of the corner block narrowed the gate: a Tet10 can be
        // corner-inverted (`det F = -1`) while every Gauss point stays strongly
        // positive, and such an element passed the whole validity gate — the
        // stretch slot abstains too, because singular values are
        // orientation-blind, so `diag(1,1,-1)` reports `max_dev = 0`.
        //
        // That state is not hypothetical: mirroring the four corners through a
        // plane and leaving the midsides inside a two-element-length box
        // produces it, and the solve returned `Ok` with the folded element in
        // it. Gating both is the fail-closed reading of Decision Q, and it
        // costs one extra 3x3 determinant per element per step boundary.
        //
        // Ordered before the reference-corner sampling below because it is the
        // cheaper test — one 3x3 determinant against four inversions — so the
        // expensive stage only ever runs on states the cheap ones accept.
        //
        // For Tet4 this is redundant by construction — `G == 1` and the single
        // Gauss pair IS the corner block — so the verdict cannot change there.
        // `grad_x_n` is the 4x3 corner block, so pair it with the element's
        // first four rows — the corner nodes, which both element types order first.
        let x_corners: SMatrix<f64, 4, 3> = x_nodes.fixed_rows::<4>(0).into_owned();
        let det_corner = deformation_gradient(&x_corners, &geom.grad_x_n).determinant();
        if !det_corner.is_finite() || det_corner <= 0.0 {
            return Err(SolverFailure::ValidityViolation {
                tet_id,
                message: format!(
                    "validity violation at tet {tet_id}: inversion = det F = \
                     {det_corner:.3} on the corner block (every Gauss point is \
                     positive) violates RequireOrientation handler (must be \
                     finite and strictly positive). Phase 4 scope memo \
                     Decision Q fail-closed semantics."
                ),
            });
        }
        // (c) The element's reference CORNERS, for a higher-order element.
        //
        // The Gauss points are interior — the four Stroud points sit at
        // barycentric weight 0.5854 on their nearest vertex, i.e. a barycentric
        // gap of 0.4146 to it — so a fold confined to a corner region is
        // invisible to them. Measured on the `corner_inverted_tet10` fixture:
        // `det F` reads +169.7 to +172.8 at the Gauss points and -1520.6 to
        // -1737.2 at the four reference corners, negative over 18.6% of the
        // reference element.
        //
        // Skipped for a linear element: `Tet4::shape_gradients` discards its
        // `xi`, so `F` is constant over the element and the single Gauss point
        // already IS this sample. Evaluating it four more times would cost four
        // 3x3 inversions per element to reproduce a number we hold.
        //
        // ⚠ Two assumptions live in this block, neither enforced by the type
        // system. `Element<N, G>` is a PUBLIC trait, so a downstream impl could
        // violate both:
        //   * `N > 4` is used as a proxy for "higher order than linear";
        //   * the four xi below assume a SIMPLEX reference domain with corner 0
        //     at the origin, which is what `tet10.rs` and `tet4.rs` use.
        // The crate is tet-only everywhere else (`TetId`, `tet_vertices`,
        // `element_node_ids`'s hard-coded 4-corners-then-6-midsides split), so
        // this is consistent with the rest rather than newly restrictive — but a
        // hex or wedge element would silently sample non-corners here.
        //
        // Not cached, deliberately. `grad_xi`, `j_rest` and `j_inv` at these four
        // points depend only on REST geometry and could be precomputed alongside
        // `GaussGeometry`. Measured cost is ~1.5 kflop/element against a
        // factorization that is 77.9% of a solve and runs per Newton iteration,
        // where this runs twice per step — well under noise. Caching is the
        // natural refactor if that ever stops being true; it would also let the
        // degenerate-rest case below be decided once at construction rather than
        // re-decided identically on every call.
        //
        // Measured worth, on 4000 states with the six midsides perturbed by
        // sigma = 18 mm on a 100 mm tet, against a dense interior sample as
        // ground truth (3456 of them hide a genuine negative `det F`):
        //
        //     Gauss points only ................ 60.0% missed
        //     + corner block ................... 60.0% missed
        //     + reference corners ..............  6.0% missed
        //
        // ⚠ Note the middle row: the corner block adds NOTHING on this
        // population, because these states perturb only midsides and the corner
        // block reads only corners. The two checks catch disjoint classes — the
        // corner block catches corner-mirrored states (what `main` gated all
        // along), this stage catches midside-driven folds near a vertex. Neither
        // subsumes the other, which is why both are here.
        //
        // ⚠ Two caveats on that table, both understating the residual risk.
        // The dense ground-truth lattice INCLUDES the reference corners, so it is
        // not independent of the stage it scores: a state whose only negativity
        // is exactly at a corner is counted as negative AND guaranteed caught.
        // (The Gauss points are not lattice points, so stage (a) has no such
        // coupling.) And the whole population is STRAIGHT-edged, so the curved
        // path — where the degenerate-rest case above lives — is unmeasured.
        //
        // ⚠ 6% is not 0%. This makes the gate a denser SAMPLING, not a proof:
        // `det F` is a polynomial in `xi` for a curved element and nothing here
        // bounds it between the sample points, so a fold living strictly between
        // samples still passes. A positivity certificate (Bernstein bounds on the
        // Jacobian) is the real answer and is not attempted here.
        if N > 4 {
            let x_rest = self.mesh.positions();
            let x_ref: SMatrix<f64, N, 3> = SMatrix::from_fn(|a, k| x_rest[nodes[a] as usize][k]);
            // The four vertices of the reference tetrahedron.
            for (c, xi) in [
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ]
            .into_iter()
            .enumerate()
            {
                let grad_xi = self.element.shape_gradients(xi);
                let j_rest = x_ref.transpose() * grad_xi;
                // ⚠ Guard the SIGN, not just invertibility.
                //
                // The constructor does NOT reject this matrix. It checks two
                // different ones: the corner edge-vector Jacobian `j_0`, and the
                // isoparametric Jacobian at the four GAUSS points. The
                // isoparametric Jacobian at a reference CORNER is neither — the
                // midsides contribute there (at `xi = 0`, `dN_4/dxi = [4,0,0]`,
                // so a column is `(X1-X0) + 4(X4 - midpoint)`), so a midside
                // projected far enough off its edge degenerates it while `j_0`
                // and all four Gauss Jacobians stay healthy. That is reachable
                // on a conformed mesh, where midsides are projected onto an SDF.
                //
                // `det j_rest < 0` matters more than `det j_rest == 0`: zero is
                // measure-zero and `try_inverse` catches it, whereas negative is
                // an OPEN set that inverts fine and then sign-flips
                // `det F = det J_def / det j_rest` — manufacturing a
                // `ValidityViolation` that blames the STATE for malformed REST
                // geometry. Skipping is fail-open at one sample point, which is
                // the lesser error: this gate polices the deformed state, and
                // rest-geometry validity belongs to the mesher.
                let det_j_rest = j_rest.determinant();
                if !det_j_rest.is_finite() || det_j_rest <= 0.0 {
                    continue;
                }
                let Some(j_inv) = j_rest.try_inverse() else {
                    continue;
                };
                let det_f = deformation_gradient(&x_nodes, &(grad_xi * j_inv)).determinant();
                if !det_f.is_finite() || det_f <= 0.0 {
                    return Err(SolverFailure::ValidityViolation {
                        tet_id,
                        message: format!(
                            "validity violation at tet {tet_id}: inversion = det F = \
                             {det_f:.3} at reference corner {c} (0-based) of 4 \
                             violates RequireOrientation handler (must be finite and \
                             strictly positive). Phase 4 scope memo Decision Q \
                             fail-closed semantics."
                        ),
                    });
                }
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
                // of the step-start check above. Without this,
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
    /// why `Material::first_piola` returns `NaN` on an `F` with `det F < 0`
    /// rather than panicking: a trial state may legitimately overshoot into
    /// inversion, the
    /// sufficient-decrease test below is false for `NaN` (every comparison
    /// against `NaN` is), so control reaches `alpha *= 0.5` and the next trial
    /// is rebuilt from `x_curr` — `assemble_global_int_force` re-zeroes `f_int`
    /// and `residual_into` rewrites every index, so the `NaN` cannot persist
    /// across trials. Panicking in the material would turn each such trial into
    /// an aborted solve.
    ///
    /// ⚠ **`det F == 0` exactly is the exception, and it does panic.**
    /// `invert_transpose`'s `try_inverse` tests `determinant().is_zero()` — an
    /// exact comparison in nalgebra's 3x3 path, not a tolerance — so a trial
    /// that lands a node exactly on the plane of its element's other three
    /// aborts the solve from inside the material. (A `NaN` determinant is not
    /// zero, so `NaN` states still flow through as described above.) The
    /// halving argument therefore covers `det F < 0`, which is the overwhelmingly
    /// likely overshoot, and not the measure-zero coplanar case.
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
