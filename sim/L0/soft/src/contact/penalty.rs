//! Penalty rigid↔soft contact — first force-bearing
//! [`ContactModel`] impl on `sim-soft`.
//!
//! One-way coupling (rigid kinematic; soft side feels the force).
//! Penalty is a stepping stone to IPC, not a production baseline. The
//! structural failure modes documented in book Part 4 §00 §00 —
//! active-set discontinuity at `d = d̂`, parameter sensitivity,
//! oscillation pathology near the boundary — stay valid; §00 §00's
//! "no penalty even as a baseline" commitment is narrowed to "no
//! penalty as a *production* baseline" (this implementation is
//! deliberately scoped to rigid↔soft co-sim plumbing, first-time
//! [`ContactModel`] wiring into the Newton hot path, and the Hertzian
//! sphere↔plane analytic gate at `tests/hertz_sphere_plane.rs`).
//!
//! The active-set-discontinuity pathology specifically is mitigated
//! (not removed) by two orthogonal optional mechanisms:
//!
//! - `PenaltyRigidContact::smoothing_eps_m` — a quintic-Hermite
//!   taper of the **gap function** over `[d̂, d̂+ε]` that makes the
//!   per-pair Hessian C⁰ across the active boundary. See
//!   `docs/CANDIDATE_C_SMOOTHED_CONTACT_SPEC.md` for the design.
//! - `PenaltyRigidContact::normal_avg_k` +
//!   `PenaltyRigidContact::normal_avg_radius_m` — per-query
//!   averaging of the contact **normal direction** over `k` axis-
//!   aligned offset samples, smoothing slope discontinuities in
//!   `n = ∂d/∂p` at SDF partition boundaries (grid cell faces for
//!   the `GridSdf` consumer in cf-device-design, or triangle Voronoi
//!   cells for mesh-direct SDFs). See
//!   `docs/CANDIDATE_E_B_NORMAL_AVERAGING_SPEC.md` for the design.
//!
//! The two compose multiplicatively in the assembled Hessian (one
//! smooths the scalar gap-function magnitude, the other smooths the
//! `n ⊗ n` outer-product direction). Both are bit-equal-when-dormant
//! (the existing constructors initialize them to disabled state).
//!
//! ## Formula
//!
//! For a soft vertex at position `p` and a rigid primitive with
//! signed-distance `d = d(p)` and outward unit normal `n = ∂d/∂p`:
//!
//! - **Energy** `E(p) = ½ κ (d̂ − d)²` for `d < d̂`, else `0`.
//! - **Gradient** `∂E/∂p = −κ (d̂ − d) · n` for `d < d̂`, else zero —
//!   at active config (`d < d̂`), `(d̂ − d) > 0` so the gradient points
//!   *opposite* the outward normal, i.e., into the rigid surface,
//!   mirroring the elastic case where `f_int` points further into the
//!   deformed configuration. The restoring force on the vertex is
//!   `−∂E/∂p = +κ (d̂ − d) · n`, along `+n` — back into the half-space
//!   outside the rigid body.
//! - **Hessian** `∂²E/∂p² = κ · n ⊗ n` (rank-1 outer product) for
//!   `d < d̂`, else zero. PSD only — two zero eigenvalues along the
//!   tangent plane to `n`, one positive eigenvalue `κ` along `n`. The
//!   full system tangent becomes SPD after the elastic Hessian is
//!   added. When `normal_avg_k` (private field on `PenaltyRigidContact`)
//!   `> 1`, `n` in the gradient / Hessian is replaced by the
//!   per-query averaged normal `n_avg` (still unit, still rank-1
//!   outer-product PSD; the eigenstructure of the assembled Hessian
//!   gains smoothness across active-set flips).
//! - **CCD** — penalty has no time-of-impact concept; `ccd_toi`
//!   returns [`f64::INFINITY`]. Phase H IPC delivers proper CCD.

use super::{ContactGradient, ContactHessian, ContactModel, ContactPair, ContactPairReadout};
use crate::{
    Vec3,
    mesh::{Mesh, VertexId},
    sdf_bridge::Sdf,
};
use nalgebra::{Matrix3, Point3};

/// Default penalty stiffness (N/m). Middle of the recommended
/// `1e3..1e5` range — at Ecoflex-class material `E ≈ 200 kPa` and
/// `h ≈ 5 mm` element edge, element stiffness `E·h ≈ 1e3 N/m`; κ at
/// 10× element stiffness is the "stiff but Newton-convergent" regime.
/// Fixtures may tune locally via [`PenaltyRigidContact::with_params`]
/// (see `tests/penalty_compressive_block.rs` and
/// `tests/hertz_sphere_plane.rs`).
pub(crate) const PENALTY_KAPPA_DEFAULT: f64 = 1.0e4;

/// Default contact band (m). 1 mm — ~20× the expected Hertz
/// indentation `δ ≈ 5e-5 m` at the canonical R = 1 cm soft sphere ×
/// Ecoflex-class composite, so the active-set band cleanly contains
/// the contact patch without pulling in spurious distant-vertex
/// pairs.
pub(crate) const PENALTY_DHAT_DEFAULT: f64 = 1.0e-3;

/// Quintic Hermite ramp `R(sd)` for the one-sided smoothing window
/// `[d_hat, d_hat + eps]`, evaluated together with its first two
/// derivatives w.r.t. `sd`. Returns `(R, dR/dsd, d²R/dsd²)`.
///
/// The polynomial is `p(τ) = 1 − 10τ³ + 15τ⁴ − 6τ⁵` over `τ ∈ [0, 1]`
/// with `τ = (sd − d̂) / ε`. Boundary conditions are `(1, 0, 0)` at
/// `τ = 0` and `(0, 0, 0)` at `τ = 1` (value / 1st / 2nd derivative);
/// both endpoints are C² so the smoothed energy
/// `E_smooth = 0.5 κ (d̂ − sd)² R(sd)` is C² at the active-pair
/// boundary. `p'(τ) = −30 τ² (1 − τ)²` is strictly negative on
/// `(0, 1)` so the ramp is monotonically decreasing in the transition
/// band.
///
/// Outside the transition band the ramp is constant: `1` for
/// `sd ≤ d̂` (full penalty), `0` for `sd ≥ d̂ + ε` (inactive). The
/// derivatives are zero in both flat regions — needed for the C²
/// contract at the transition-band endpoints to extend across the
/// region boundaries.
///
/// Requires `eps > 0` — the smoothing-window-empty case
/// (`smoothing_eps_m == 0.0`) is handled at the call site
/// ([`PenaltyRigidContact::pair_contribution`]'s `sd <= d_hat` branch)
/// without invoking this helper.
fn quintic_ramp(sd: f64, d_hat: f64, eps: f64) -> (f64, f64, f64) {
    debug_assert!(
        eps > 0.0 && eps.is_finite(),
        "quintic_ramp called with non-positive eps {eps}",
    );
    if sd <= d_hat {
        return (1.0, 0.0, 0.0);
    }
    if sd >= d_hat + eps {
        return (0.0, 0.0, 0.0);
    }
    let tau = (sd - d_hat) / eps;
    let tau2 = tau * tau;
    let tau3 = tau2 * tau;
    let one_minus_tau = 1.0 - tau;
    // p(τ) = 1 − 10τ³ + 15τ⁴ − 6τ⁵
    //      = 1 + τ³·(−10 + τ·(15 − 6τ)) — Horner from highest degree.
    let p = (-6.0_f64)
        .mul_add(tau, 15.0) // 15 − 6τ
        .mul_add(tau, -10.0) // −10 + τ·(15 − 6τ)
        .mul_add(tau3, 1.0); // 1 + τ³·(…)
    // p'(τ) = −30·τ²·(1 − τ)² — strictly ≤ 0 on [0, 1], so the ramp
    // is monotonically decreasing in the transition band.
    let dp_dtau = -30.0 * tau2 * one_minus_tau * one_minus_tau;
    // p''(τ) = −60·τ·(1 − τ)·(1 − 2τ) — zero at τ = 0, 1/2, 1; the
    // 1/2 root is the inflection inside the band.
    let one_minus_two_tau = (-2.0_f64).mul_add(tau, 1.0);
    let d2p_dtau2 = -60.0 * tau * one_minus_tau * one_minus_two_tau;
    (p, dp_dtau / eps, d2p_dtau2 / (eps * eps))
}

/// Penalty contact between soft-body vertices and a set of kinematic
/// rigid primitives.
///
/// One [`ContactPair::Vertex`] per `(soft vertex, rigid primitive)`
/// whose signed distance is below the contact band `d̂`. See the
/// [module docs](self) for the energy / gradient / Hessian formulas
/// and sign conventions.
///
/// Primitives are heap-erased [`Sdf`] trait objects so a single contact
/// model can compose mixed primitive types (planes, spheres,
/// scan-derived `mesh_sdf::Signed<TriMeshDistance, _>`, cf-design `Solid`s)
/// in one list. The constructors are generic over any `IntoIterator`
/// of `Sdf + 'static`, so a homogeneous `Vec<RigidPlane>` flows
/// through without explicit boxing.
pub struct PenaltyRigidContact {
    primitives: Vec<Box<dyn Sdf>>,
    kappa: f64,
    d_hat: f64,
    /// Optional active-set lower bound. When `Some(c)`, the active-set
    /// walk excludes pairs whose signed distance is below `-c` (the
    /// pathologically-deep-interior band). `None` (default) preserves
    /// the original behavior — every pair in the contact band fires
    /// (`sd < d̂` when [`smoothing_eps_m`](Self::smoothing_eps_m) is
    /// zero, `sd < d̂ + smoothing_eps_m` when smoothing is on).
    ///
    /// Use case: rigid-vs-soft sims where the rigid SDF is defined
    /// globally over a closed-body domain (e.g., a flood-filled
    /// `GridSdf` of a cleaned body-part scan), and a moving pose can
    /// project soft-body vertices into deep-interior regions where the
    /// SDF reports `sd << 0`. Without the filter, those pairs generate
    /// force `κ·(d̂ − sd)·n` with `(d̂ − sd)` in tens of millimeters,
    /// breaking Newton convergence (Armijo stall). With the filter,
    /// those pairs are silently skipped; elastic forces can move the
    /// vertex back into the contact band, at which point normal
    /// penalty contact resumes.
    ///
    /// **Scope of the filter**: applies at the active-set walk only
    /// ([`ActivePairsFor::active_pairs`](super::ActivePairsFor::active_pairs)
    /// and [`Self::per_pair_readout`]). The pair-indexed
    /// [`ContactModel`](super::ContactModel) methods (`energy` /
    /// `gradient` / `hessian`) do NOT re-check the cutoff and would
    /// return the unfiltered penalty values if called directly with a
    /// deep-interior pair. This is fine in practice because the solver
    /// only invokes pair-indexed methods on pairs `active_pairs`
    /// produced. The asymmetry is explicit so a future consumer that
    /// hand-builds pairs is aware of the contract.
    ///
    /// Motivating consumer: the `cf-device-design` sliding-intruder FEM
    /// ramp (`docs/SIM_ARC_SLIDING_INTRUDER_CONTACT_RECON.md`).
    interior_cutoff: Option<f64>,
    /// One-sided smoothing window (m) above `d_hat`. When `> 0`, pairs
    /// with `sd ∈ (d_hat, d_hat + smoothing_eps_m)` contribute a
    /// quintic-Hermite-tapered penalty that smoothly reaches 0 at
    /// `sd = d_hat + smoothing_eps_m`. Addresses class-2 active-set
    /// chattering in `H_contact(x)` (see
    /// `docs/CANDIDATE_C_SMOOTHED_CONTACT_SPEC.md`).
    ///
    /// `0.0` (default) — hard penalty, bit-equal to pre-candidate-C
    /// behavior. Existing constructors ([`new`](Self::new),
    /// [`with_params`](Self::with_params),
    /// [`with_params_and_interior_cutoff`](Self::with_params_and_interior_cutoff))
    /// initialize this to `0.0`; the
    /// [`with_params_and_smoothing`](Self::with_params_and_smoothing) and
    /// [`with_params_and_smoothing_and_interior_cutoff`](Self::with_params_and_smoothing_and_interior_cutoff)
    /// constructors opt in.
    ///
    /// The smoothed energy is `0.5 · κ · (d̂ − sd)² · R(sd)` where
    /// `R(sd) = p((sd − d̂) / ε)` with `p(τ) = 1 − 10τ³ + 15τ⁴ − 6τ⁵`
    /// — quintic Hermite with boundary conditions `(1, 0, 0)` at
    /// `τ = 0` and `(0, 0, 0)` at `τ = 1`. Both endpoint value /
    /// derivative / second-derivative are zero so the energy is C²
    /// across the active-pair boundary and the assembled Hessian
    /// (dropping the SDF-curvature ∂n/∂p term, see module docs) is C⁰
    /// — the chattering fix.
    smoothing_eps_m: f64,
    /// Number of offset samples for per-query contact-normal averaging
    /// (F3 recon B candidate E.b). `1` (default) disables averaging —
    /// `n = prim.grad(p)` bit-equal to pre-E.b behavior. `7` enables
    /// the 6-face axis-aligned neighborhood
    /// (`prim.grad(p ± r·e_{x,y,z})` averaged with `prim.grad(p)`).
    /// Addresses class-2 active-set chattering via normal-direction
    /// smoothing — orthogonal to [`smoothing_eps_m`](Self::smoothing_eps_m)
    /// which smooths the gap function instead. See
    /// `docs/CANDIDATE_E_B_NORMAL_AVERAGING_SPEC.md` for the design.
    normal_avg_k: u8,
    /// Offset radius (m) for per-query contact-normal averaging.
    /// Only consulted when [`normal_avg_k`](Self::normal_avg_k) `> 1`.
    /// Default `0.0` (matching the disabled state). For `k > 1`,
    /// constructors require strictly positive + finite.
    normal_avg_radius_m: f64,
}

/// Per-pair scalar contributions (energy, `dE/dsd`, `d²E/dsd²`) at a
/// given signed distance, before composition with the per-pair normal.
/// `gradient = dE/dsd · n` and `hessian = d²E/dsd² · n ⊗ n` (the
/// latter dropping the SDF-curvature ∂n/∂p term, consistent with the
/// existing hard-penalty Hessian — see module docstring).
///
/// Returned by [`PenaltyRigidContact::pair_contribution`]; consumed by
/// the four `(per_pair_readout, energy, gradient, hessian)` sites that
/// need formula values (`active_pairs` only needs the gate, so uses
/// [`PenaltyRigidContact::pair_is_active`] directly).
struct PairContribution {
    energy: f64,
    d_energy_d_sd: f64,
    d2_energy_d_sd2: f64,
}

impl PenaltyRigidContact {
    /// Construct with default `(κ, d̂)` from `PENALTY_KAPPA_DEFAULT` /
    /// `PENALTY_DHAT_DEFAULT` (crate-private; tunable only via
    /// [`with_params`](Self::with_params) for fixture-local tuning).
    #[must_use]
    pub fn new<I>(primitives: I) -> Self
    where
        I: IntoIterator,
        I::Item: Sdf + 'static,
    {
        Self::with_params(primitives, PENALTY_KAPPA_DEFAULT, PENALTY_DHAT_DEFAULT)
    }

    /// Construct with non-default `(κ, d̂)` — testing surface for
    /// fixture-local overrides (see
    /// `tests/penalty_compressive_block.rs`,
    /// `tests/hertz_sphere_plane.rs`, `tests/contact_grad_hook.rs`).
    /// User-facing parameter tuning is a future-phase follow-on;
    /// production scenes go through [`new`](Self::new).
    #[must_use]
    pub fn with_params<I>(primitives: I, kappa: f64, d_hat: f64) -> Self
    where
        I: IntoIterator,
        I::Item: Sdf + 'static,
    {
        let primitives: Vec<Box<dyn Sdf>> = primitives
            .into_iter()
            .map(|p| Box::new(p) as Box<dyn Sdf>)
            .collect();
        Self {
            primitives,
            kappa,
            d_hat,
            interior_cutoff: None,
            smoothing_eps_m: 0.0,
            normal_avg_k: 1,
            normal_avg_radius_m: 0.0,
        }
    }

    /// Construct with non-default `(κ, d̂)` plus a positive interior
    /// cutoff — pairs whose signed distance is below `-interior_cutoff`
    /// are silently excluded from the active set. See the
    /// `interior_cutoff` (private) field docs for the
    /// motivating use case.
    ///
    /// # Panics
    ///
    /// `interior_cutoff` must be (1) strictly positive + finite, and
    /// (2) strictly greater than `d_hat` — so the active band
    /// `[−cutoff, d_hat)` is non-empty + contains legitimate
    /// near-contact pairs.
    #[must_use]
    pub fn with_params_and_interior_cutoff<I>(
        primitives: I,
        kappa: f64,
        d_hat: f64,
        interior_cutoff: f64,
    ) -> Self
    where
        I: IntoIterator,
        I::Item: Sdf + 'static,
    {
        assert!(
            interior_cutoff > 0.0 && interior_cutoff.is_finite(),
            "interior_cutoff must be positive and finite, got {interior_cutoff}",
        );
        assert!(
            interior_cutoff > d_hat,
            "interior_cutoff ({interior_cutoff}) must be > d_hat ({d_hat}) — \
             so the active band [−cutoff, d_hat) is non-empty",
        );
        let primitives: Vec<Box<dyn Sdf>> = primitives
            .into_iter()
            .map(|p| Box::new(p) as Box<dyn Sdf>)
            .collect();
        Self {
            primitives,
            kappa,
            d_hat,
            interior_cutoff: Some(interior_cutoff),
            smoothing_eps_m: 0.0,
            normal_avg_k: 1,
            normal_avg_radius_m: 0.0,
        }
    }

    /// Construct with non-default `(κ, d̂)` plus a positive smoothing
    /// window `smoothing_eps_m` above `d̂`. See the
    /// `smoothing_eps_m` (private) field docs for the
    /// formula + motivating use case + bit-equal-when-dormant contract.
    ///
    /// # Panics
    ///
    /// `smoothing_eps_m` must be (1) non-negative + finite. Passing
    /// `0.0` is allowed (equivalent to [`with_params`](Self::with_params)
    /// — caller may bind a single constructor symbol when smoothing
    /// is configured externally).
    #[must_use]
    pub fn with_params_and_smoothing<I>(
        primitives: I,
        kappa: f64,
        d_hat: f64,
        smoothing_eps_m: f64,
    ) -> Self
    where
        I: IntoIterator,
        I::Item: Sdf + 'static,
    {
        assert!(
            smoothing_eps_m >= 0.0 && smoothing_eps_m.is_finite(),
            "smoothing_eps_m must be non-negative and finite, got {smoothing_eps_m}",
        );
        let primitives: Vec<Box<dyn Sdf>> = primitives
            .into_iter()
            .map(|p| Box::new(p) as Box<dyn Sdf>)
            .collect();
        Self {
            primitives,
            kappa,
            d_hat,
            interior_cutoff: None,
            smoothing_eps_m,
            normal_avg_k: 1,
            normal_avg_radius_m: 0.0,
        }
    }

    /// Construct with non-default `(κ, d̂)` plus a positive smoothing
    /// window and a positive interior cutoff. Composes
    /// [`with_params_and_smoothing`](Self::with_params_and_smoothing) and
    /// [`with_params_and_interior_cutoff`](Self::with_params_and_interior_cutoff).
    /// Motivating consumer: the `cf-device-design` sliding-intruder
    /// FEM ramp (needs the interior cutoff for pose-dependent
    /// deep-interior pairs and the smoothing window for class-2
    /// chattering at deep cavity insets).
    ///
    /// # Panics
    ///
    /// `smoothing_eps_m` must be non-negative + finite (see
    /// [`with_params_and_smoothing`](Self::with_params_and_smoothing)).
    /// `interior_cutoff` must be strictly positive + finite + strictly
    /// greater than `d_hat + smoothing_eps_m` — so the active band
    /// `[−cutoff, d_hat + smoothing_eps_m)` is non-empty and contains
    /// legitimate near-contact pairs.
    #[must_use]
    pub fn with_params_and_smoothing_and_interior_cutoff<I>(
        primitives: I,
        kappa: f64,
        d_hat: f64,
        smoothing_eps_m: f64,
        interior_cutoff: f64,
    ) -> Self
    where
        I: IntoIterator,
        I::Item: Sdf + 'static,
    {
        assert!(
            smoothing_eps_m >= 0.0 && smoothing_eps_m.is_finite(),
            "smoothing_eps_m must be non-negative and finite, got {smoothing_eps_m}",
        );
        assert!(
            interior_cutoff > 0.0 && interior_cutoff.is_finite(),
            "interior_cutoff must be positive and finite, got {interior_cutoff}",
        );
        assert!(
            interior_cutoff > d_hat + smoothing_eps_m,
            "interior_cutoff ({interior_cutoff}) must be > d_hat + smoothing_eps_m \
             ({}) — so the active band [−cutoff, d_hat + smoothing_eps_m) is non-empty",
            d_hat + smoothing_eps_m,
        );
        let primitives: Vec<Box<dyn Sdf>> = primitives
            .into_iter()
            .map(|p| Box::new(p) as Box<dyn Sdf>)
            .collect();
        Self {
            primitives,
            kappa,
            d_hat,
            interior_cutoff: Some(interior_cutoff),
            smoothing_eps_m,
            normal_avg_k: 1,
            normal_avg_radius_m: 0.0,
        }
    }

    /// Construct with non-default `(κ, d̂)` plus a positive smoothing
    /// window AND per-query normal averaging (F3 recon B candidate
    /// E.b). Composes
    /// [`with_params_and_smoothing`](Self::with_params_and_smoothing)
    /// and the normal-averaging machinery. Smoothing and normal-
    /// averaging are orthogonal axes (one smooths the gap function,
    /// the other smooths the contact normal direction).
    ///
    /// # Panics
    ///
    /// `smoothing_eps_m` must be non-negative + finite (see
    /// [`with_params_and_smoothing`](Self::with_params_and_smoothing)).
    /// `normal_avg_k` must be in `{1, 7}` (iter-1 closed-set —
    /// `{19, 27}` reserved for §7 fallback in the spec; future
    /// discretizations extend the assertion).
    /// `normal_avg_radius_m` must be non-negative + finite, and
    /// strictly positive iff `normal_avg_k > 1` — the
    /// `(k > 1, r == 0)` combo is a silent no-op + likely a caller
    /// mistake.
    #[must_use]
    pub fn with_params_and_smoothing_and_normal_averaging<I>(
        primitives: I,
        kappa: f64,
        d_hat: f64,
        smoothing_eps_m: f64,
        normal_avg_k: u8,
        normal_avg_radius_m: f64,
    ) -> Self
    where
        I: IntoIterator,
        I::Item: Sdf + 'static,
    {
        assert!(
            smoothing_eps_m >= 0.0 && smoothing_eps_m.is_finite(),
            "smoothing_eps_m must be non-negative and finite, got {smoothing_eps_m}",
        );
        assert!(
            matches!(normal_avg_k, 1 | 7),
            "normal_avg_k must be 1 (disabled) or 7 (6-face axis-aligned), got {normal_avg_k}",
        );
        assert!(
            normal_avg_radius_m >= 0.0 && normal_avg_radius_m.is_finite(),
            "normal_avg_radius_m must be non-negative and finite, got {normal_avg_radius_m}",
        );
        assert!(
            normal_avg_k == 1 || normal_avg_radius_m > 0.0,
            "normal_avg_radius_m must be strictly positive when normal_avg_k > 1, got k = {normal_avg_k}, r = {normal_avg_radius_m}",
        );
        let primitives: Vec<Box<dyn Sdf>> = primitives
            .into_iter()
            .map(|p| Box::new(p) as Box<dyn Sdf>)
            .collect();
        Self {
            primitives,
            kappa,
            d_hat,
            interior_cutoff: None,
            smoothing_eps_m,
            normal_avg_k,
            normal_avg_radius_m,
        }
    }

    /// Construct with non-default `(κ, d̂)` plus a positive smoothing
    /// window, per-query normal averaging, AND a positive interior
    /// cutoff (F3 recon B candidate E.b). The full sliding-intruder
    /// FEM ramp constructor — composes
    /// [`with_params_and_smoothing_and_normal_averaging`](Self::with_params_and_smoothing_and_normal_averaging)
    /// with
    /// [`with_params_and_interior_cutoff`](Self::with_params_and_interior_cutoff).
    ///
    /// # Panics
    ///
    /// Combined panics of the composed constructors: `smoothing_eps_m`
    /// non-negative + finite; `normal_avg_k ∈ {1, 7}`;
    /// `normal_avg_radius_m` non-negative + finite + strictly positive
    /// iff `k > 1`; `interior_cutoff` strictly positive + finite +
    /// strictly greater than `d_hat + smoothing_eps_m`.
    #[must_use]
    pub fn with_params_and_smoothing_and_normal_averaging_and_interior_cutoff<I>(
        primitives: I,
        kappa: f64,
        d_hat: f64,
        smoothing_eps_m: f64,
        normal_avg_k: u8,
        normal_avg_radius_m: f64,
        interior_cutoff: f64,
    ) -> Self
    where
        I: IntoIterator,
        I::Item: Sdf + 'static,
    {
        assert!(
            smoothing_eps_m >= 0.0 && smoothing_eps_m.is_finite(),
            "smoothing_eps_m must be non-negative and finite, got {smoothing_eps_m}",
        );
        assert!(
            matches!(normal_avg_k, 1 | 7),
            "normal_avg_k must be 1 (disabled) or 7 (6-face axis-aligned), got {normal_avg_k}",
        );
        assert!(
            normal_avg_radius_m >= 0.0 && normal_avg_radius_m.is_finite(),
            "normal_avg_radius_m must be non-negative and finite, got {normal_avg_radius_m}",
        );
        assert!(
            normal_avg_k == 1 || normal_avg_radius_m > 0.0,
            "normal_avg_radius_m must be strictly positive when normal_avg_k > 1, got k = {normal_avg_k}, r = {normal_avg_radius_m}",
        );
        assert!(
            interior_cutoff > 0.0 && interior_cutoff.is_finite(),
            "interior_cutoff must be positive and finite, got {interior_cutoff}",
        );
        assert!(
            interior_cutoff > d_hat + smoothing_eps_m,
            "interior_cutoff ({interior_cutoff}) must be > d_hat + smoothing_eps_m \
             ({}) — so the active band [−cutoff, d_hat + smoothing_eps_m) is non-empty",
            d_hat + smoothing_eps_m,
        );
        let primitives: Vec<Box<dyn Sdf>> = primitives
            .into_iter()
            .map(|p| Box::new(p) as Box<dyn Sdf>)
            .collect();
        Self {
            primitives,
            kappa,
            d_hat,
            interior_cutoff: Some(interior_cutoff),
            smoothing_eps_m,
            normal_avg_k,
            normal_avg_radius_m,
        }
    }

    /// Per-pair contact-normal computation — single source of truth
    /// for the three consumer sites that need a unit normal at a
    /// contact pair query point.
    ///
    /// At `normal_avg_k == 1` (default) returns `prim.grad(p)`
    /// bit-equal — preserves pre-E.b behavior under the construction
    /// path that doesn't opt into averaging. At `k > 1` averages over
    /// the center sample `prim.grad(p)` plus `k - 1` axis-aligned
    /// offset samples at radius `normal_avg_radius_m`, then
    /// renormalizes the sum. For `k == 7`, the 6 offset directions are
    /// `±e_x, ±e_y, ±e_z`.
    ///
    /// **Fallback chain**: if the renormalization sum has norm
    /// `< 1e-10` (deeply pathological alignment), falls back to the
    /// center grad; if that is also degenerate, falls back to
    /// `Vec3::z()` — matching `gradient_clamped`'s degenerate-grad
    /// posture in `cf-geometry::sdf::SdfGrid` (the `GridSdf` consumer).
    ///
    /// **MAINTENANCE NOTE** — any future change to the averaging
    /// discretization (offset directions, weights, fallback policy)
    /// updates this method only. The three consumer sites
    /// ([`PenaltyRigidContact::per_pair_readout`],
    /// [`ContactModel::gradient`], [`ContactModel::hessian`]) call
    /// this helper rather than `prim.grad` directly. See §2.6 of
    /// `docs/CANDIDATE_E_B_NORMAL_AVERAGING_SPEC.md` for the mirror-
    /// on-change-prevention rationale (parallel to
    /// [`Self::pair_is_active`]'s MAINTENANCE NOTE for the active-
    /// band predicate).
    fn averaged_normal(&self, prim: &dyn Sdf, p: Point3<f64>) -> Vec3 {
        let center = prim.grad(p);
        if self.normal_avg_k == 1 {
            return center;
        }
        debug_assert_eq!(
            self.normal_avg_k, 7,
            "normal_avg_k validated to {{1, 7}} in constructors",
        );
        let r = self.normal_avg_radius_m;
        // 6-face axis-aligned offsets for k = 7. Future {k = 19, 27}
        // discretizations append to this match per spec §2.2.
        let offsets: [Vec3; 6] = [
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(-r, 0.0, 0.0),
            Vec3::new(0.0, r, 0.0),
            Vec3::new(0.0, -r, 0.0),
            Vec3::new(0.0, 0.0, r),
            Vec3::new(0.0, 0.0, -r),
        ];
        let mut sum = center;
        for off in &offsets {
            sum += prim.grad(p + off);
        }
        let norm = sum.norm();
        if norm > 1e-10 {
            sum / norm
        } else if center.norm() > 1e-10 {
            center
        } else {
            Vec3::z()
        }
    }

    /// Pair-inclusion gate — the single source of truth for the
    /// active-band predicate. Returns `true` iff `sd` is below the
    /// interior cutoff (when set) and below `d_hat + smoothing_eps_m`
    /// (the upper edge of the active band, including any smoothing
    /// window).
    ///
    /// **MAINTENANCE NOTE** — any future change to the active-band
    /// extent (e.g., extending it further, or shifting it relative to
    /// `d_hat`) updates this method only. The five consumer sites
    /// ([`PenaltyRigidContact::per_pair_readout`],
    /// [`ContactModel::energy`], [`ContactModel::gradient`],
    /// [`ContactModel::hessian`] —
    /// via [`Self::pair_contribution`] which delegates to this gate —
    /// and [`ActivePairsFor::active_pairs`] — directly) inherit the
    /// change for free. See §2.5 of
    /// `docs/CANDIDATE_C_SMOOTHED_CONTACT_SPEC.md` for the
    /// mirror-on-change-prevention rationale.
    fn pair_is_active(&self, sd: f64) -> bool {
        if let Some(c) = self.interior_cutoff
            && sd < -c
        {
            return false;
        }
        sd < self.d_hat + self.smoothing_eps_m
    }

    /// Per-pair scalar contributions at the given signed distance, or
    /// `None` if the pair is outside the active band. Returns the same
    /// gate decision as [`Self::pair_is_active`] — composing the two
    /// methods on the same `sd` is sound, but `pair_is_active` is the
    /// cheap path for `active_pairs` walks that don't need the formula.
    ///
    /// When `smoothing_eps_m == 0.0`, the returned scalars at any
    /// `sd < d_hat` are bit-equal to the pre-smoothing hard-penalty
    /// formula (see `docs/CANDIDATE_C_SMOOTHED_CONTACT_SPEC.md` §3 for
    /// the bit-equal-when-dormant contract).
    fn pair_contribution(&self, sd: f64) -> Option<PairContribution> {
        if !self.pair_is_active(sd) {
            return None;
        }
        // sd is in the active band. Two sub-branches: full penalty
        // (sd <= d_hat) — always bit-equal to the hard penalty
        // including when smoothing is on — versus quintic-tapered
        // (sd ∈ (d_hat, d_hat + smoothing_eps_m), only reachable when
        // smoothing_eps_m > 0).
        let gap = self.d_hat - sd;
        if sd <= self.d_hat {
            return Some(PairContribution {
                energy: 0.5 * self.kappa * gap * gap,
                d_energy_d_sd: -self.kappa * gap,
                d2_energy_d_sd2: self.kappa,
            });
        }
        let (r, r_p, r_pp) = quintic_ramp(sd, self.d_hat, self.smoothing_eps_m);
        // gap < 0 in this branch (sd > d_hat). gap² is still positive.
        let gap_sq = gap * gap;
        // dE/dsd = 0.5κ·[−2·gap·R + gap²·R'(sd)]
        let d_energy_d_sd = 0.5 * self.kappa * gap_sq.mul_add(r_p, -2.0 * gap * r);
        // d²E/dsd² = 0.5κ·[2R − 4·gap·R'(sd) + gap²·R''(sd)]
        let d2_energy_d_sd2 =
            0.5 * self.kappa * gap_sq.mul_add(r_pp, 2.0_f64.mul_add(r, -4.0 * gap * r_p));
        Some(PairContribution {
            energy: 0.5 * self.kappa * gap_sq * r,
            d_energy_d_sd,
            d2_energy_d_sd2,
        })
    }

    /// Per-active-pair readout — for every `(soft vertex, rigid
    /// primitive)` pair in the contact band, emit a
    /// [`ContactPairReadout`] with the vertex position, signed distance,
    /// outward primitive normal, and the penalty force on the soft side
    /// at the readout-time `positions`.
    ///
    /// Mirrors [`active_pairs`](super::ActivePairsFor::active_pairs)'s walk order
    /// (vertices outer × primitives inner) and band gate (via
    /// `Self::pair_is_active` — private), so the returned vec is the same length
    /// as `active_pairs(...)` at the same `positions` and the readouts
    /// appear in the same order.
    ///
    /// For the hard-penalty case (`smoothing_eps_m == 0.0`),
    /// `force_on_soft` resolves to `+κ·(d̂ − sd)·n` per the type docs'
    /// sign convention — a bit-equivalent reproduction of the energy
    /// gradient [`ContactModel::gradient`] returns (`−κ·(d̂ − sd)·n`),
    /// negated by the force-as-`−∇U` identity. With smoothing, the
    /// force in the transition band `(d̂, d̂+ε)` follows from the
    /// quintic-tapered energy (see
    /// `Self::smoothing_eps_m` (private) field docs).
    /// Row 18 (`contact-force-readout`) is the canonical consumer;
    /// row 14 (`compressive-block`) reconstructs this surface inline
    /// from known plane geometry, predating this method.
    // `vid as VertexId` and `pid as u32` mirror `active_pairs`'s `Vec`-
    // iteration index packing — bounded by mesh / primitive counts that
    // fit in `u32` for any Phase 5 scene.
    #[allow(clippy::cast_possible_truncation)]
    #[must_use]
    pub fn per_pair_readout<M: crate::material::Material>(
        &self,
        _mesh: &dyn Mesh<M>,
        positions: &[Vec3],
    ) -> Vec<ContactPairReadout> {
        let mut readouts = Vec::new();
        for (vid, &p) in positions.iter().enumerate() {
            let p_pt = Point3::from(p);
            for (pid, prim) in self.primitives.iter().enumerate() {
                let sd = prim.eval(p_pt);
                if let Some(contribution) = self.pair_contribution(sd) {
                    let normal = self.averaged_normal(prim.as_ref(), p_pt);
                    let force_on_soft = -contribution.d_energy_d_sd * normal;
                    readouts.push(ContactPairReadout {
                        pair: ContactPair::Vertex {
                            vertex_id: vid as VertexId,
                            primitive_id: pid as u32,
                        },
                        position: p,
                        sd,
                        normal,
                        force_on_soft,
                    });
                }
            }
        }
        readouts
    }
}

/// Filter `per_pair_readout` results to vertices in the given referenced
/// set, dropping orphan BCC lattice corners that are not in any tet.
///
/// [`PenaltyRigidContact::per_pair_readout`] returns one
/// [`ContactPairReadout`] for every body vertex in the contact band
/// (`sd < d̂`) regardless of whether that vertex is referenced by any
/// tet. Orphans sit at their rest BCC-lattice positions and are
/// excluded from the solver's free-DOF set; their inclusion in
/// readouts is a deterministic regression gate but pollutes
/// physically-meaningful aggregates. For probe-inside-cavity
/// geometries the orphan share can dominate at 95-97 % of readouts
/// (rows 21 + 22 silicone-sleeve precedent — see pattern (xx) at
/// `project_sim_soft_row_22_patterns.md`).
///
/// Returns a fresh `Vec` containing only the referenced-vertex readouts
/// in the same order as the input. Pass `referenced_vertices(&mesh)`
/// (or any subset thereof) for the `referenced` slice.
#[must_use]
pub fn filter_pair_readouts_to_referenced(
    readouts: Vec<ContactPairReadout>,
    referenced: &[VertexId],
) -> Vec<ContactPairReadout> {
    let set: std::collections::BTreeSet<VertexId> = referenced.iter().copied().collect();
    readouts
        .into_iter()
        .filter(|r| match r.pair {
            ContactPair::Vertex { vertex_id, .. } => set.contains(&vertex_id),
        })
        .collect()
}

impl ContactModel for PenaltyRigidContact {
    fn energy(&self, pair: &ContactPair, positions: &[Vec3]) -> f64 {
        let &ContactPair::Vertex {
            vertex_id,
            primitive_id,
        } = pair;
        let p = Point3::from(positions[vertex_id as usize]);
        let d = self.primitives[primitive_id as usize].eval(p);
        self.pair_contribution(d).map_or(0.0, |c| c.energy)
    }

    fn gradient(&self, pair: &ContactPair, positions: &[Vec3]) -> ContactGradient {
        let &ContactPair::Vertex {
            vertex_id,
            primitive_id,
        } = pair;
        let p = Point3::from(positions[vertex_id as usize]);
        let prim = &self.primitives[primitive_id as usize];
        let d = prim.eval(p);
        self.pair_contribution(d)
            .map_or_else(ContactGradient::default, |c| {
                let n = self.averaged_normal(prim.as_ref(), p);
                let force = c.d_energy_d_sd * n;
                ContactGradient {
                    contributions: vec![(vertex_id, force)],
                }
            })
    }

    fn hessian(&self, pair: &ContactPair, positions: &[Vec3]) -> ContactHessian {
        let &ContactPair::Vertex {
            vertex_id,
            primitive_id,
        } = pair;
        let p = Point3::from(positions[vertex_id as usize]);
        let prim = &self.primitives[primitive_id as usize];
        let d = prim.eval(p);
        self.pair_contribution(d)
            .map_or_else(ContactHessian::default, |c| {
                let n = self.averaged_normal(prim.as_ref(), p);
                let block: Matrix3<f64> = c.d2_energy_d_sd2 * (n * n.transpose());
                ContactHessian {
                    contributions: vec![(vertex_id, vertex_id, block)],
                }
            })
    }

    fn ccd_toi(&self, _pair: &ContactPair, _x0: &[Vec3], _x1: &[Vec3]) -> f64 {
        f64::INFINITY
    }
}

impl<M: crate::material::Material> super::ActivePairsFor<M> for PenaltyRigidContact {
    /// Walks soft vertices outer (`0..positions.len()`) × rigid
    /// primitives inner (`0..self.primitives.len()`); emits a
    /// [`ContactPair::Vertex`] for every `(v, p)` whose signed
    /// distance is in the active band per
    /// `PenaltyRigidContact::pair_is_active` (private)
    /// (`sd < d̂ + smoothing_eps_m` and above the interior cutoff when
    /// set). Order is deterministic — no sort, no `HashMap`, no rayon.
    // `vid as VertexId` and `pid as u32` are `Vec`-iteration indices;
    // in practice bounded by mesh / primitive counts that fit
    // comfortably in `u32`. The `as` cast matches the convention used
    // in `mesh/hand_built.rs` for `VertexId` packing. A theoretical
    // overflow on a 64-bit pointer would surface as wrapped indices;
    // not load-bearing for Phase 5 mesh sizes.
    #[allow(clippy::cast_possible_truncation)]
    fn active_pairs(&self, _mesh: &dyn Mesh<M>, positions: &[Vec3]) -> Vec<ContactPair> {
        let mut pairs = Vec::new();
        for (vid, &p) in positions.iter().enumerate() {
            let p_pt = Point3::from(p);
            for (pid, prim) in self.primitives.iter().enumerate() {
                let sd = prim.eval(p_pt);
                if self.pair_is_active(sd) {
                    pairs.push(ContactPair::Vertex {
                        vertex_id: vid as VertexId,
                        primitive_id: pid as u32,
                    });
                }
            }
        }
        pairs
    }
}
