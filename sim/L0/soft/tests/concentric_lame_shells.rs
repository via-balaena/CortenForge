//! IV-5 — three-shell concentric Lamé end-to-end through the full
//! `SdfMeshedTetMesh` pipeline.
//!
//! Phase 4 scope memo §1 IV-5 + §8 commit 11 + Decision D (two
//! load-bearing analytic regression tests; IV-3 on the hand-built
//! cantilever-bilayer beam is the FEM-machinery gate, IV-5 here is the
//! end-to-end SDF→FEM gate). Three-shell concentric hollow silicone
//! sphere meshed through the full BCC + Labelle-Shewchuk pipeline,
//! internal pressure on the cavity wall, fixed Dirichlet pin on the
//! outer surface; the Saint-Venant-averaged radial displacement on the
//! cavity wall is compared against the closed-form piecewise-Lamé
//! prediction at three refinement levels with monotone error reduction.
//!
//! ## Why fixed-outer-surface, not free-outer
//!
//! Internal-pressure inflation on a free-floating sphere has six
//! rigid-body modes (3 translation + 3 rotation) the linear system
//! cannot disambiguate without an explicit anchor. Three options
//! exist:
//!
//! 1. **Pin a sparse 3-or-4 vertex tetrahedron at the outer surface.**
//!    Removes the 6 modes uniquely; introduces local Saint-Venant
//!    distortion at the pin points; breaks the spherical-symmetry of
//!    the analytic closed-form.
//! 2. **Use the inertial term `M / Δt²` as a regulariser.** At the
//!    walking-skeleton `Δt = 1e-2` the inertial regularisation is
//!    enough to make the tangent stiffness non-singular, but the
//!    static-equilibrium solution requires either explicit damping or
//!    a multi-step rollout.
//! 3. **Pin the entire outer surface (full Dirichlet on every
//!    R_outer-band vertex).** Kills all 6 rigid-body modes by
//!    construction with perfect spherical symmetry; the closed-form
//!    Lamé multi-shell solution generalises cleanly to the
//!    fixed-outer-surface BC variant via `u_r^{(N)}(R_b) = 0`
//!    replacing the standard `σ_rr^{(N)}(R_b) = 0`. The 6×6 linear
//!    system rebuilds once with the swapped row.
//!
//! IV-5 takes (3). The static-pressure inflation problem with a fixed
//! outer surface is mechanically meaningful (a thick-walled sphere
//! anchored at its outer skin under internal pressurization — the
//! soft-body analogue of a rigid-shell-encased pressure vessel), and
//! the analytic comparison stays a clean closed-form derivation rather
//! than a Saint-Venant-region carve-out.
//!
//! ## Compressible Neo-Hookean regime — Decision J deviation per IV-3
//!
//! Decision J prescribes Ecoflex 00-30 with `ν ≈ 0.49` (near-
//! incompressible) for the canonical layered-silicone-device. IV-3
//! (commit 8, `bonded_bilayer_beam.rs:42-56`) deviates to `ν = 0.4`
//! (compressible Neo-Hookean, `λ = 4 μ`) because Tet4 cantilever-
//! bending under `ν → 0.5` exhibits volumetric locking
//! (Part 2 §05 §00, `~100×` over-stiffening as `ν → 0.5`).
//!
//! The same deviation applies to IV-5 here, for analogous but distinct
//! reason. Internal-pressure inflation under `ν → 0.5` is a different
//! locking signature: the radial-expansion mode `u_r ∝ 1/r²` in the
//! incompressible limit is volume-preserving, and a Tet4 element's
//! single-Gauss-point integration cannot represent a per-element
//! volume-preserving deformation while simultaneously matching the
//! global radial displacement profile. The over-determined per-tet
//! incompressibility constraint suppresses the deviatoric expansion
//! modes the analytic Lamé solution lives in, and convergence
//! collapses. Phase H Tet10 + F-bar recovers the near-incompressible
//! Ecoflex regime per Part 2 §05 §02.
//!
//! Material parameters across the three shells use distinct
//! `(μ, λ)` pairs in the same compressible family — outer Ecoflex 00-30
//! at `1×` baseline, middle composite at the `2×` Decision J
//! multiplier (Part 1 §04 §02), inner Ecoflex 00-30 at the same `1×`
//! baseline. All three shells share `λ = 4 μ` ⇒ `ν = 0.4`, matching
//! the IV-1 / IV-2 / IV-3 baseline family for cross-commit
//! consistency. The outer-and-inner-equal-stiffness symmetry is the
//! Decision J commitment (the device's outer / middle / inner are
//! Ecoflex / composite / Ecoflex); IV-4 (commit 10) deviated from this
//! symmetry for shell-swap-bug discrimination (`0.5× / 1× / 2×`), but
//! IV-5's analytic-comparison gate doesn't depend on per-shell
//! discrimination — it asserts the displacement field matches the
//! piecewise-Lamé prediction across all three layers, which the
//! 1×/2×/1× pair tests exactly.
//!
//! ## Closed-form three-shell Lamé under internal pressure with fixed
//! outer surface
//!
//! Per shell `i ∈ {1, 2, 3}` (innermost first), radial displacement
//! and radial stress under spherical symmetry:
//!
//! ```text
//! u_r^{(i)}(r) = A_i · r + B_i / r²
//! σ_rr^{(i)}(r) = (3 λ_i + 2 μ_i) A_i − 4 μ_i B_i / r³
//! ```
//!
//! Six unknowns `(A_1, B_1, A_2, B_2, A_3, B_3)`; six boundary /
//! continuity conditions:
//!
//! 1. `σ_rr^{(1)}(R_a) = -p` — inner-cavity traction (compression sign).
//! 2. `u_r^{(3)}(R_b) = 0` — FIXED outer surface (replaces the standard
//!    free-surface `σ_rr = 0` BC).
//! 3. `u_r^{(1)}(R_1) = u_r^{(2)}(R_1)` — displacement continuity at
//!    the inner / middle interface.
//! 4. `σ_rr^{(1)}(R_1) = σ_rr^{(2)}(R_1)` — radial stress continuity.
//! 5. `u_r^{(2)}(R_2) = u_r^{(3)}(R_2)` — displacement continuity at
//!    the middle / outer interface.
//! 6. `σ_rr^{(2)}(R_2) = σ_rr^{(3)}(R_2)` — radial stress continuity.
//!
//! Six linear equations in six unknowns; assemble as a
//! [`nalgebra::Matrix6<f64>`] and solve via partial-pivot LU. The 6×6
//! is well-conditioned at the chosen radii (ratios `1.5 / 1.33 / 1.25`
//! all in the `[1.2, 2.5]` rule-of-thumb band); a one-line `SVD`
//! condition-number check at test setup gates `κ(M) < 1e12` to surface
//! any future radii change that erodes conditioning. The bound is
//! loose because empirical `κ` at the IV-5 radii is `O(1e8)` due to
//! the wide A-vs-B coefficient scale gap (`A_i` rows ~ `K_i` ~ `1e6`,
//! `B_i` rows ~ `μ / r³` ~ `1e10`); the gate detects collapse, not
//! marginal degradation.
//!
//! Reference: Timoshenko & Goodier, *Theory of Elasticity*, 3rd ed., §141
//! (thick-walled spherical pressure vessel — single-shell closed-form);
//! Fung, *Foundations of Solid Mechanics*, Ch. 5 (multi-layer
//! continuity). The three-shell + fixed-outer system is a routine
//! generalisation of the single-shell free-outer Timoshenko closed-form
//! and stays in the linear-elastic regime where Neo-Hookean reduces to
//! Lamé at small strain (Part 2 Ch 04 §00). IV-5's chosen pressure
//! lives well inside that regime (~`1 %` cavity-radius inflation, see
//! [`PRESSURE`] doc).
//!
//! ## Saint-Venant averaging at the cavity wall
//!
//! The mesh's cavity-surface vertices are not perfectly distributed —
//! the BCC lattice imposes a discrete cut-pattern that lands more
//! cut-points near cube corners than near cube faces. Reading the
//! cavity-wall displacement at any single vertex would conflate
//! per-vertex BCC quantisation with the FEM convergence rate.
//! Saint-Venant averaging — the canonical FEM idiom for distributed-
//! load-and-symmetric-geometry problems — sums radial displacements
//! over every cavity-surface vertex and divides by the count, giving a
//! single mean `u_r(R_cavity)` value to compare against the analytic
//! `u_r^{(1)}(R_a)`. Per-vertex variance is suppressed by the average;
//! the residual mesh-discretisation error then converges as the rate
//! the IV-5 convergence-test assertion targets.
//!
//! ## Static regime
//!
//! Mirroring IV-3 (`bonded_bilayer_beam.rs:130-146`):
//! `cfg.dt = STATIC_DT = 1.0 s` collapses the inertial term `M / Δt²`
//! by `~4` orders of magnitude relative to stiffness; a single
//! `replay_step` from rest converges to static equilibrium at
//! `tol = 1e-10` far below the round-off floor.
//!
//! Empirically Newton converges in `~3` iters per level — far below
//! IV-3's `~15-25` because the radially-symmetric solution under
//! distributed pressure with a fixed-outer-pin BC is in the linearisable
//! small-strain regime from rest (no rotations, no tip-force
//! concentration, ~1 % inflation). `cfg.max_newton_iter = 50` mirrors
//! IV-3 verbatim for headroom against future load / material
//! perturbations and against the convergence-test's per-level iter-cap
//! sanity bound (`< 40`).
//!
//! ## Convergence assertion
//!
//! Three refinements `cell_size ∈ {0.04, 0.02, 0.01}` mirror the IV-4
//! refinement scan exactly (commit 10's IV-4 region-tagging gate runs
//! the same three levels on the same canonical sphere geometry). At the
//! finest level the body has `~57 000` tets; the static `replay_step`
//! at that resolution stays inside the per-test runtime budget per
//! [`feedback_release_mode_heavy_tests`].
//!
//! Asserted shape — bound chosen as a conservative regression-detector
//! ceiling, looser than IV-3's `~O(h^1.5)` for two distinct reasons:
//!
//! 1. **Pre-implementation prediction.** IV-3 uses a HAND-BUILT mesh
//!    with the bilayer interface aligned to a tet-face plane
//!    (`z = HEIGHT / 2` is exactly a `nz`-grid level), so the per-
//!    element stiffness assignment is exact at the interface and the
//!    convergence rate reflects pure FEM discretisation of the
//!    displacement field. IV-5 uses the SDF mesher with bonded
//!    interfaces (Decision E: centroid-tag with misclassification-band,
//!    no mesh-aligned interfaces); the per-tet material assignment is
//!    correct in the bulk of each shell but misclassified in a thin
//!    band of straddler-tets at each interface, with the
//!    misclassification fraction shrinking as `O(h)` per IV-4
//!    (commit 10). The pre-impl prediction was that the integrated
//!    displacement-field error would track the same `O(h)` rate,
//!    dominating the FEM-discretisation contribution.
//! 2. **Empirical observation.** With the IV-5 chosen geometry +
//!    Decision J 1× / 2× / 1× stiffness pattern + fixed-outer-pin BC,
//!    the convergence rate at the fine end is empirically
//!    super-quadratic (rate `~3.5`, ratio `~0.09`), nowhere near `O(h)`.
//!    The radially-symmetric Lamé solution has no localised high-
//!    gradient features that the misclassification-band typically
//!    matters for, and the moderate stiffness contrast (factor 2)
//!    keeps interface-band integral error below FEM-discretisation
//!    error. The `0.55` bound was set under the (1) prediction; it
//!    stays as a conservative regression-detector floor against future
//!    parameter / mesher perturbations that could erode convergence.
//!    Phase H interface-aware refinement remains the path to strict
//!    O(h²) recovery for harder configurations (high stiffness
//!    contrast, non-radial geometry).
//!
//! Asserted shape:
//! - **Monotone error reduction:** `err_h > err_h2 > err_h4`.
//! - **Fine-end ratio:** `err_h4 ≤ 0.55 · err_h2` (rate `≥ ~0.86`).
//! - **Coarse-to-fine improvement:** the rate at `h/2 → h/4` exceeds
//!   the rate at `h → h/2`, demonstrating approach-to-asymptote.
//! - **Absolute floor:** `err_h4 / analytic < 0.20`. Looser than IV-3's
//!   `0.15` for the same misclassification-dominant reason — at the
//!   straddler-band, integrated stiffness contrast contributes
//!   directly to the FEM displacement integral.
//!
//! ## Test layout
//!
//! Three `#[test]` blocks mirroring the IV-4 (commit 10) cadence:
//!
//! 1. [`iv_5_uniform_passthrough_at_h2_matches_single_shell_lame`] —
//!    sanity: at refinement `cell_size = 0.02`, a uniform-middle
//!    `MaterialField` (every tet at the middle composite's `(μ, λ)`)
//!    collapses the three-shell scene to a single-material thick-
//!    walled hollow sphere. The closed-form 6×6 reduces to the
//!    single-shell Lamé closed-form at the same `(μ, λ)`. Catches
//!    constructor / BC / SDF-sign-convention / load-direction bugs
//!    before the convergence test trusts those moving parts. Asserts
//!    relative error within `0.30` — the Tet4 + centroid-tag
//!    discretisation band at `h/2` per the IV-3 sanity-band
//!    precedent.
//!
//! 2. [`iv_5_three_shell_converges_to_piecewise_lame`] — main: three
//!    refinements (h, h/2, h/4), full three-shell field, assert
//!    monotone error reduction + fine-end ratio `< 0.55` + rate-
//!    improvement + absolute floor `< 0.20`.
//!
//! 3. [`iv_5_three_shell_is_run_to_run_deterministic`] — Decision N
//!    determinism: two builds at `h/2` produce bit-equal `materials()`
//!    cache and bit-equal `step.x_final` after a single `replay_step`.

#![allow(
    // Direct displacement comparisons drive the sanity-band and
    // convergence-ratio assertions; the floats are engineering numbers
    // with documented analytic baselines, not numerical bit patterns
    // (so `clippy::float_cmp` would flag them on every line below).
    clippy::float_cmp,
    // The closed-form 6×6 setup tallies a dozen named-constant scalars;
    // engineering test prose reads more cleanly with the names locally
    // derived than with a hoisted const block split across the file.
    clippy::many_single_char_names,
    // Saint-Venant-averaged radial displacement is `Σ u_r / N`, the
    // canonical FEM idiom for distributed-load reading. clippy's
    // "use a sum + division" lint is exactly that idiom.
    clippy::cast_precision_loss,
    // `from_sdf` calls `.expect()` to surface meshing failures as test
    // panics — the canonical layered-silicone-sphere scene either
    // succeeds by construction or surfaces a regression worth
    // investigating (mirrors III-1 + IV-4 conventions).
    clippy::expect_used
)]

use nalgebra::{Matrix6, Vector6};
use sim_soft::{
    CpuNewtonSolver, CpuTet4NHSolver, Field, LAYERED_SPHERE_R_CAVITY, LAYERED_SPHERE_R_INNER_OUTER,
    LAYERED_SPHERE_R_OUTER, LAYERED_SPHERE_R_OUTER_INNER, LayeredScalarField, MaterialField, Mesh,
    NullContact, SceneInitial, SdfMeshedTetMesh, SoftScene, Solver, SolverConfig, SphereSdf, Tet4,
    Vec3,
};

mod common;

use common::assert_neo_hookean_bit_equal;

// ── Material parameters per Decision J (compressible regime) ─────────────

/// Outer Ecoflex 00-30 baseline, compressible Lamé pair (`λ = 4 μ` ⇒
/// `ν = 0.4`) per the module docstring's "Compressible Neo-Hookean
/// regime" section. Same baseline IV-1 / IV-2 / IV-3 anchor on.
const MU_OUTER: f64 = 1.0e5;
const LAMBDA_OUTER: f64 = 4.0e5;

/// Middle carbon-black composite shell: `2×` stiffness multiplier per
/// Decision J (Part 1 §04 §02). Same `ν = 0.4` (both Lamé parameters
/// scale together).
const MU_MIDDLE: f64 = 2.0e5;
const LAMBDA_MIDDLE: f64 = 8.0e5;

/// Inner Ecoflex 00-30 — same baseline as outer per Decision J's
/// outer / middle / inner = Ecoflex / composite / Ecoflex commitment.
const MU_INNER: f64 = 1.0e5;
const LAMBDA_INNER: f64 = 4.0e5;

// ── Internal pressure ────────────────────────────────────────────────────

/// Internal pressure on the cavity wall (Pa). At the chosen radii and
/// material parameters the Lamé closed-form predicts cavity-wall radial
/// displacement `u_r(R_cavity) ≈ 4.6e-4 m` ≈ `1.2 %` of `R_cavity = 0.04
/// m` — comfortably small-strain, well inside the linear-elastic regime
/// where Neo-Hookean reduces to Lamé. The pressure value is engineered
/// to land in this window:
///
/// ```text
/// inflation_target = ~1 %       (small-strain, below ~3 % where Neo-
///                                Hookean's nonlinear contribution
///                                exceeds 5-digit linear approximation)
/// E_outer ≈ 2.8e5 Pa            (at μ=1e5, λ=4e5, ν=0.4)
/// p ≈ 5e3 Pa                    (chosen to land inflation in target)
/// ```
const PRESSURE: f64 = 5.0e3;

// ── Refinement scan ──────────────────────────────────────────────────────

/// Coarsest cell size — matches IV-4 (commit 10) baseline.
const CELL_SIZE_H: f64 = 0.04;

/// Mid refinement — matches IV-4 (commit 10) and the canonical Phase 3
/// commit 9 scene cell size.
const CELL_SIZE_H2: f64 = 0.02;

/// Fine refinement — matches IV-4 (commit 10). At ~57 000 tets the
/// pressure-inflation `replay_step` stays inside the per-test runtime
/// budget per `feedback_release_mode_heavy_tests`.
const CELL_SIZE_H4: f64 = 0.01;

// ── Static-regime time step ──────────────────────────────────────────────

/// Mirrors IV-3 (`bonded_bilayer_beam.rs:STATIC_DT`). At `dt = 1.0` the
/// inertial term `M / dt²` collapses by `~4` orders of magnitude
/// relative to the stiffness contribution, so a single `replay_step`
/// from rest converges to the static equilibrium at `tol = 1e-10`. The
/// outer-surface Dirichlet pin makes the tangent stiffness non-singular
/// even at the static limit (no rigid-body modes).
const STATIC_DT: f64 = 1.0;

// ── MaterialField builders ───────────────────────────────────────────────

/// Three-shell `MaterialField` per Decision J: outer Ecoflex / middle
/// composite / inner Ecoflex. Mirrors IV-4's three-shell construction
/// pattern (`sdf_material_tagging.rs:three_shell_field`) — same SDF
/// reuse-as-partition trick, same `partition_point(|&t| t <= phi)`
/// boundary convention.
fn three_shell_field() -> MaterialField {
    let body = || {
        Box::new(SphereSdf {
            radius: LAYERED_SPHERE_R_OUTER,
        })
    };
    // Threshold list is on `phi = ‖x‖ − R_OUTER`. At `‖x‖ < R_INNER_OUTER`
    // the inner Ecoflex shell wins (phi < -(R_OUTER - R_INNER_OUTER)).
    let phi_inner_outer = LAYERED_SPHERE_R_INNER_OUTER - LAYERED_SPHERE_R_OUTER;
    let phi_outer_inner = LAYERED_SPHERE_R_OUTER_INNER - LAYERED_SPHERE_R_OUTER;
    let mu_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        body(),
        vec![phi_inner_outer, phi_outer_inner],
        vec![MU_INNER, MU_MIDDLE, MU_OUTER],
    ));
    let lambda_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        body(),
        vec![phi_inner_outer, phi_outer_inner],
        vec![LAMBDA_INNER, LAMBDA_MIDDLE, LAMBDA_OUTER],
    ));
    MaterialField::from_fields(mu_field, lambda_field)
}

/// Uniform-middle `MaterialField` for the sanity-passthrough test. Every
/// tet receives the middle-composite Lamé pair, collapsing the three-
/// shell scene to a single-material thick-walled hollow sphere.
fn uniform_middle_field() -> MaterialField {
    MaterialField::uniform(MU_MIDDLE, LAMBDA_MIDDLE)
}

// ── Closed-form Lamé multi-shell solver ──────────────────────────────────

/// Per-shell parameter pair `(μ, λ)`. The closed-form solver below
/// indexes shells inner-to-outer.
#[derive(Clone, Copy)]
struct ShellParams {
    mu: f64,
    lambda: f64,
}

impl ShellParams {
    /// `K = 3 λ + 2 μ` — the radial-stress coefficient on `A_i` per
    /// shell (see module docstring's `σ_rr` formula).
    fn k_coefficient(self) -> f64 {
        3.0_f64.mul_add(self.lambda, 2.0 * self.mu)
    }
}

/// Closed-form multi-shell `(A_i, B_i)` coefficients for the radial-
/// displacement form `u_r^{(i)}(r) = A_i r + B_i / r²`.
///
/// Six unknowns laid out as `[A_1, B_1, A_2, B_2, A_3, B_3]`. See
/// module docstring for the 6×6 system derivation.
struct LameCoefficients {
    a: [f64; 3],
    b: [f64; 3],
}

impl LameCoefficients {
    /// Evaluate `u_r^{(i)}(r) = A_i r + B_i / r²`.
    fn u_r(&self, shell_index: usize, r: f64) -> f64 {
        self.a[shell_index].mul_add(r, self.b[shell_index] / (r * r))
    }
}

/// Solve the 6×6 closed-form system for the three-shell hollow sphere
/// under internal pressure `p` at `R_a` with **fixed outer surface**
/// (`u_r^{(3)}(R_b) = 0`) and bonded continuity at `R_1` and `R_2`.
///
/// The matrix `M` is well-conditioned at the chosen IV-5 radii; an
/// inline SVD-based κ-bound check (see body, post-assembly) gates
/// `κ(M) < 1e12` at test setup to surface any future radii change that
/// collapses invertibility. See module docstring's "Closed-form three-
/// shell Lamé" section for the κ-bound rationale.
fn solve_three_shell_lame(
    shells: [ShellParams; 3],
    radii: [f64; 4], // [R_a, R_1, R_2, R_b]
    pressure: f64,
) -> LameCoefficients {
    let r_a = radii[0];
    let r_1 = radii[1];
    let r_2 = radii[2];
    let r_b = radii[3];
    let k = [
        shells[0].k_coefficient(),
        shells[1].k_coefficient(),
        shells[2].k_coefficient(),
    ];
    let mu = [shells[0].mu, shells[1].mu, shells[2].mu];

    // Layout: x = [A_1, B_1, A_2, B_2, A_3, B_3]ᵀ.
    let mut m = Matrix6::zeros();
    let mut b_vec = Vector6::zeros();

    // Eq 1: σ_rr^{(1)}(R_a) = -p
    //       K_1 A_1 - 4 μ_1 B_1 / R_a³ = -p
    m[(0, 0)] = k[0];
    m[(0, 1)] = -4.0 * mu[0] / r_a.powi(3);
    b_vec[0] = -pressure;

    // Eq 2: u_r^{(3)}(R_b) = 0  (fixed outer surface)
    //       A_3 R_b + B_3 / R_b² = 0
    m[(1, 4)] = r_b;
    m[(1, 5)] = 1.0 / (r_b * r_b);
    b_vec[1] = 0.0;

    // Eq 3: u_r^{(1)}(R_1) = u_r^{(2)}(R_1)
    //       A_1 R_1 + B_1 / R_1² - A_2 R_1 - B_2 / R_1² = 0
    m[(2, 0)] = r_1;
    m[(2, 1)] = 1.0 / (r_1 * r_1);
    m[(2, 2)] = -r_1;
    m[(2, 3)] = -1.0 / (r_1 * r_1);
    b_vec[2] = 0.0;

    // Eq 4: σ_rr^{(1)}(R_1) = σ_rr^{(2)}(R_1)
    //       K_1 A_1 - 4 μ_1 B_1 / R_1³ - K_2 A_2 + 4 μ_2 B_2 / R_1³ = 0
    m[(3, 0)] = k[0];
    m[(3, 1)] = -4.0 * mu[0] / r_1.powi(3);
    m[(3, 2)] = -k[1];
    m[(3, 3)] = 4.0 * mu[1] / r_1.powi(3);
    b_vec[3] = 0.0;

    // Eq 5: u_r^{(2)}(R_2) = u_r^{(3)}(R_2)
    //       A_2 R_2 + B_2 / R_2² - A_3 R_2 - B_3 / R_2² = 0
    m[(4, 2)] = r_2;
    m[(4, 3)] = 1.0 / (r_2 * r_2);
    m[(4, 4)] = -r_2;
    m[(4, 5)] = -1.0 / (r_2 * r_2);
    b_vec[4] = 0.0;

    // Eq 6: σ_rr^{(2)}(R_2) = σ_rr^{(3)}(R_2)
    //       K_2 A_2 - 4 μ_2 B_2 / R_2³ - K_3 A_3 + 4 μ_3 B_3 / R_2³ = 0
    m[(5, 2)] = k[1];
    m[(5, 3)] = -4.0 * mu[1] / r_2.powi(3);
    m[(5, 4)] = -k[2];
    m[(5, 5)] = 4.0 * mu[2] / r_2.powi(3);
    b_vec[5] = 0.0;

    // Conditioning gate — surfaces any future radii change that erodes
    // the matrix's invertibility. Empirical κ at the IV-5 radii is
    // O(1e8) due to the wide A-vs-B coefficient scale gap (`A_i` rows
    // ~ K_i ~ 1e6, `B_i` rows ~ μ / r³ ~ 1e10); 1e12 leaves margin
    // against parameter perturbation while still loud-detecting a
    // collapsed system.
    let cond = m.svd(false, false).singular_values;
    let kappa = cond[0] / cond[5];
    assert!(
        kappa < 1.0e12,
        "Lamé 6×6 conditioning κ(M) = {kappa:e} ≥ 1e12 at radii \
         (R_a, R_1, R_2, R_b) = ({r_a}, {r_1}, {r_2}, {r_b}); \
         radii ratios outside the well-conditioned band [1.2, 2.5] erode \
         invertibility — verify shell-radii choice before trusting the \
         analytic prediction"
    );

    let solution = m.lu().solve(&b_vec).expect(
        "Lamé 6×6 LU solve failed — matrix is singular or near-singular; check shell-radii \
         conditioning gate above",
    );
    LameCoefficients {
        a: [solution[0], solution[2], solution[4]],
        b: [solution[1], solution[3], solution[5]],
    }
}

/// Helper for the sanity test: collapse to single-shell Lamé under a
/// uniform `(μ, λ)` field. Reuses [`solve_three_shell_lame`] with all
/// three shells at the same parameters — by symmetry the closed-form
/// reproduces the single-shell solution at every `r ∈ [R_a, R_b]`.
fn solve_single_shell_lame_via_three_shell(
    shell: ShellParams,
    r_a: f64,
    r_b: f64,
    pressure: f64,
) -> LameCoefficients {
    solve_three_shell_lame(
        [shell, shell, shell],
        [
            r_a,
            LAYERED_SPHERE_R_INNER_OUTER,
            LAYERED_SPHERE_R_OUTER_INNER,
            r_b,
        ],
        pressure,
    )
}

// ── Test scaffold — single replay_step at static dt ──────────────────────

/// Bundled output of [`run_at_refinement`] — carries the
/// Saint-Venant-averaged cavity-wall radial displacement plus Newton
/// diagnostics for the `eprintln!` line.
struct StepReport {
    /// Mean radial displacement `Σ_v (|x_final[v]| - R_cavity_v_rest) /
    /// N_loaded` over every cavity-surface vertex, where
    /// `R_cavity_v_rest = positions_rest[v].norm()`. Saint-Venant idiom
    /// (mirror IV-3's tip-displacement read pattern).
    cavity_u_r_mean: f64,
    /// Newton iteration count at convergence.
    iter_count: usize,
    /// Free-DOF residual norm at convergence.
    residual_norm: f64,
    /// Number of cavity-surface vertices (the `N_loaded` denominator).
    n_loaded: usize,
    /// Number of pinned outer-surface vertices.
    n_pinned: usize,
    /// Total tet count at this refinement.
    n_tets: usize,
}

/// Single backward-Euler `replay_step` on the layered silicone sphere
/// at `cell_size` with the supplied `MaterialField` and internal
/// pressure. Returns the cavity-wall mean radial displacement plus
/// Newton diagnostics.
fn run_at_refinement(cell_size: f64, field: MaterialField, pressure: f64) -> StepReport {
    let (mesh, bc, initial, theta) = SoftScene::layered_silicone_sphere(field, cell_size, pressure)
        .expect("layered_silicone_sphere should mesh successfully at canonical cell sizes");

    let SceneInitial { x_prev, v_prev } = initial;

    // Snapshot the cavity-surface vertices' rest radii before the mesh
    // moves into the solver — needed for the Saint-Venant `u_r` read
    // after the step.
    let positions = mesh.positions();
    let cavity_rest: Vec<(usize, f64)> = bc
        .loaded_vertices
        .iter()
        .map(|&(v, _)| (v as usize, positions[v as usize].norm()))
        .collect();
    let n_loaded = bc.loaded_vertices.len();
    let n_pinned = bc.pinned_vertices.len();
    let n_tets = mesh.n_tets();

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    // Mirrors IV-3's static-regime headroom. Empirically Newton
    // converges in ~3 iters per level under the radially-symmetric
    // distributed-pressure load with fixed-outer-pin BC — quadratic
    // convergence kicks in immediately because the rest configuration
    // is close to the small-strain equilibrium. `50` leaves headroom
    // against future perturbations (and matches IV-3 verbatim).
    cfg.max_newton_iter = 50;

    let solver: CpuTet4NHSolver<SdfMeshedTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);
    let step = solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt);

    // Saint-Venant-averaged cavity-wall radial displacement.
    // Use `Vec3::norm` for the FMA-optimised radial-magnitude
    // computation rather than open-coding `(Σ xᵢ²).sqrt()`.
    let u_r_sum: f64 = cavity_rest
        .iter()
        .map(|&(v, rest_radius)| {
            let final_pos = Vec3::new(
                step.x_final[3 * v],
                step.x_final[3 * v + 1],
                step.x_final[3 * v + 2],
            );
            final_pos.norm() - rest_radius
        })
        .sum();
    let cavity_u_r_mean = u_r_sum / cavity_rest.len() as f64;

    StepReport {
        cavity_u_r_mean,
        iter_count: step.iter_count,
        residual_norm: step.final_residual_norm,
        n_loaded,
        n_pinned,
        n_tets,
    }
}

// ── Tests ────────────────────────────────────────────────────────────────

#[test]
fn iv_5_uniform_passthrough_at_h2_matches_single_shell_lame() {
    // Sanity: every tet at the middle composite's `(μ, λ)` collapses
    // the three-shell scene to a single-material thick-walled hollow
    // sphere; the closed-form 6×6 with `shells = [middle, middle,
    // middle]` reproduces the standard single-shell Lamé closed-form
    // by symmetry.
    //
    // The 30 % band absorbs the SDF-mesher centroid-tag discretisation
    // at `cell_size = 0.02` (IV-3 sanity-band precedent at the same
    // `0.30` rel-err — see `bonded_bilayer_beam.rs:497-526`). A passing
    // 30 % band confirms the constructor + outer-surface pin + cavity-
    // surface pressure load + uniform-field code path are wired
    // correctly; the discriminating gate is the convergence test
    // below, not this sanity bound.
    let middle = ShellParams {
        mu: MU_MIDDLE,
        lambda: LAMBDA_MIDDLE,
    };
    let coeffs = solve_single_shell_lame_via_three_shell(
        middle,
        LAYERED_SPHERE_R_CAVITY,
        LAYERED_SPHERE_R_OUTER,
        PRESSURE,
    );
    let analytic = coeffs.u_r(0, LAYERED_SPHERE_R_CAVITY);

    let report = run_at_refinement(CELL_SIZE_H2, uniform_middle_field(), PRESSURE);

    let rel_err = (report.cavity_u_r_mean - analytic).abs() / analytic;
    eprintln!(
        "iv_5 uniform-middle sanity at cell_size = {h:.3}: measured u_r(R_cavity) = {disp:e}, \
         single-shell Lamé analytic = {analytic:e}, rel_err = {rel_err:.4}, n_tets = {n_tets}, \
         n_loaded = {n_loaded}, n_pinned = {n_pinned}, newton iters = {iters}, residual = {res:e}",
        h = CELL_SIZE_H2,
        disp = report.cavity_u_r_mean,
        n_tets = report.n_tets,
        n_loaded = report.n_loaded,
        n_pinned = report.n_pinned,
        iters = report.iter_count,
        res = report.residual_norm,
    );

    assert!(
        report.cavity_u_r_mean > 0.0,
        "uniform-middle cavity wall must displace outward (got {disp:e}); a non-positive value \
         signals a broken pressure-load direction or BC plumbing",
        disp = report.cavity_u_r_mean,
    );
    assert!(
        analytic > 0.0,
        "single-shell Lamé analytic must be positive at internal pressure (got {analytic:e}); a \
         non-positive value signals a sign error in the closed-form 6×6 setup"
    );
    assert!(
        rel_err < 0.30,
        "uniform-middle passthrough at cell_size = {h:.3}: |measured - analytic| / analytic = \
         {rel_err:.4} > 0.30; measured = {disp:e}, analytic = {analytic:e}",
        h = CELL_SIZE_H2,
        disp = report.cavity_u_r_mean,
    );
}

// 100-line cap is for production code where breaking apart is a
// readability win. Engineering test prose with closure assertions +
// per-level `eprintln!` diagnostics + structured-message assertion
// blocks is more readable in one sequential body than split across
// helpers that have to thread state through tedious arg lists.
#[allow(clippy::too_many_lines)]
#[test]
fn iv_5_three_shell_converges_to_piecewise_lame() {
    // Three refinements on the full three-shell field. **Assertion
    // target: piecewise-Lamé closed-form** at the cavity wall
    // (`u_r^{(1)}(R_a)`).
    //
    // The convergence rate is dominated by the SDF mesher's centroid-
    // tag-with-misclassification-band interface treatment (Decision E:
    // bonded interfaces, no mesh alignment), which converges as `O(h)`
    // per IV-4 (commit 10). The `0.55` fine-end ratio bound (rate
    // `~0.86`) is correspondingly looser than IV-3's `0.36` (rate
    // `~1.47`) — IV-3 used a HAND-BUILT mesh with the bilayer interface
    // exactly aligned to a tet-face plane, and its convergence rate
    // reflects pure FEM-discretisation error rather than the
    // interface-misclassification band IV-5 exercises. Phase H
    // interface-aware refinement closes the gap to the FEM-only rate.
    let shells = [
        ShellParams {
            mu: MU_INNER,
            lambda: LAMBDA_INNER,
        },
        ShellParams {
            mu: MU_MIDDLE,
            lambda: LAMBDA_MIDDLE,
        },
        ShellParams {
            mu: MU_OUTER,
            lambda: LAMBDA_OUTER,
        },
    ];
    let radii = [
        LAYERED_SPHERE_R_CAVITY,
        LAYERED_SPHERE_R_INNER_OUTER,
        LAYERED_SPHERE_R_OUTER_INNER,
        LAYERED_SPHERE_R_OUTER,
    ];
    let coeffs = solve_three_shell_lame(shells, radii, PRESSURE);
    let analytic = coeffs.u_r(0, LAYERED_SPHERE_R_CAVITY);

    let report_h = run_at_refinement(CELL_SIZE_H, three_shell_field(), PRESSURE);
    let report_h2 = run_at_refinement(CELL_SIZE_H2, three_shell_field(), PRESSURE);
    let report_h4 = run_at_refinement(CELL_SIZE_H4, three_shell_field(), PRESSURE);

    let measured_h = report_h.cavity_u_r_mean;
    let measured_h2 = report_h2.cavity_u_r_mean;
    let measured_h4 = report_h4.cavity_u_r_mean;

    let err_h = (measured_h - analytic).abs();
    let err_h2 = (measured_h2 - analytic).abs();
    let err_h4 = (measured_h4 - analytic).abs();

    let rate_coarse = (err_h / err_h2).log2();
    let rate_fine = (err_h2 / err_h4).log2();

    eprintln!(
        "iv_5 convergence: piecewise-Lamé target = {analytic:e}; \
         h = {h:.3} (n_tets = {n_h}, u_r = {measured_h:e}, err = {err_h:e}, \
                     iters = {ih}, res = {rh:e}); \
         h/2 = {h2:.3} (n_tets = {n_h2}, u_r = {measured_h2:e}, err = {err_h2:e}, \
                        iters = {ih2}, res = {rh2:e}); \
         h/4 = {h4:.3} (n_tets = {n_h4}, u_r = {measured_h4:e}, err = {err_h4:e}, \
                        iters = {ih4}, res = {rh4:e}); \
         coarse rate = {rate_coarse:.3}, fine rate = {rate_fine:.3}",
        h = CELL_SIZE_H,
        h2 = CELL_SIZE_H2,
        h4 = CELL_SIZE_H4,
        n_h = report_h.n_tets,
        n_h2 = report_h2.n_tets,
        n_h4 = report_h4.n_tets,
        ih = report_h.iter_count,
        rh = report_h.residual_norm,
        ih2 = report_h2.iter_count,
        rh2 = report_h2.residual_norm,
        ih4 = report_h4.iter_count,
        rh4 = report_h4.residual_norm,
    );

    // Newton-budget sanity: each level must converge well below the
    // 50-iter cap. IV-3 precedent.
    for (label, report) in [("h", &report_h), ("h/2", &report_h2), ("h/4", &report_h4)] {
        assert!(
            report.iter_count < 40,
            "Newton at {label} ran {iters} iters, within 10 of the 50-iter cap — investigate \
             solver / load regime regression before bumping the cap",
            iters = report.iter_count,
        );
    }

    // Direction sanity: every refinement must displace outward and
    // stay in the small-strain regime. Negative or order-of-magnitude-
    // off displacement at any level signals a load-direction or
    // material-assignment bug, not a refinement issue.
    for (label, d) in [
        ("h", measured_h),
        ("h/2", measured_h2),
        ("h/4", measured_h4),
    ] {
        assert!(
            d > 0.0 && d < 0.005,
            "cavity u_r at refinement {label} = {d:e} out of plausible range \
             (expected ~5e-4 m, must be positive and small-strain)"
        );
    }

    // Monotone error reduction across refinements.
    assert!(
        err_h2 < err_h,
        "h/2 error {err_h2:e} must be smaller than h error {err_h:e}",
    );
    assert!(
        err_h4 < err_h2,
        "h/4 error {err_h4:e} must be smaller than h/2 error {err_h2:e}",
    );

    // Asymptotic rate at the fine end ≥ ~0.86 (ratio ≤ 0.55). True
    // O(h) gives ratio 0.5 (rate 1.0); 0.55 leaves margin for the
    // interface-misclassification band IV-4 (commit 10) measured at
    // straddler-fraction halving cleanly per refinement level. At
    // ratio > 0.55 we'd be below O(h^0.86) territory and the FEM-plus-
    // interface-band system is not converging at the rate IV-4's
    // monotonicity gate plus FEM-discretisation overhead implies.
    let ratio_fine = err_h4 / err_h2;
    let h4 = CELL_SIZE_H4;
    assert!(
        ratio_fine < 0.55,
        "h/4 / h/2 error ratio {ratio_fine:.4} ≥ 0.55 (rate {rate_fine:.3}); convergence at \
         fine end below ~O(h^0.86) — Tet4 + SDF-meshed bonded-interface should be inside the \
         approach-to-asymptote regime by `cell_size = {h4:.3}` (~57k tets). If this fails \
         consistently, the dominant error mode has shifted from interface-misclassification to \
         a different regime; investigate before relaxing the bound."
    );

    // Coarse-to-fine improvement: rate at h/2 → h/4 must exceed rate
    // at h → h/2. Demonstrates approach-to-asymptote (the canonical
    // SDF-mesher pattern: coarse-grid quantisation dominates at h,
    // asymptoting to O(h) as h decreases) rather than a sub-asymptotic
    // plateau.
    assert!(
        rate_fine > rate_coarse,
        "fine-end rate {rate_fine:.3} must exceed coarse-end rate {rate_coarse:.3}; the \
         convergence rate must IMPROVE as h decreases, demonstrating approach to the O(h) \
         asymptote (a flat or worsening rate signals a coarse-grid quantisation that the \
         chosen refinement plan should outrun)"
    );

    // Refinement-floor sanity: at h/4 the absolute error must be
    // small relative to the analytic prediction. Looser than IV-3's
    // 0.15 because the interface-misclassification band contributes
    // directly to the displacement-field integral, and per-shell
    // stiffness contrast (1× / 2× / 1×) adds discrete jumps the
    // centroid-tagged tet treatment cannot smooth.
    let rel_err_h4 = err_h4 / analytic;
    assert!(
        rel_err_h4 < 0.20,
        "h/4 relative error {rel_err_h4:.4} ≥ 0.20; convergence ratios passed but the \
         absolute floor is too high — piecewise-Lamé prediction probably misaligned with what \
         the FEM converges to (check radii constants, Decision J Lamé pairs, or BC sign)"
    );
}

// Parallel `a` / `b` naming on the determinism test's two-build
// pattern (mesh_a / mesh_b, theta_a / theta_b, step_a / step_b) is the
// idiomatic shape for bit-equality determinism tests across `sim-soft`
// and is what makes the parallelism visible at a glance. clippy's
// `similar_names` heuristic flags single-letter suffix differences as
// confusing; in this test the parallelism IS the intent.
#[allow(clippy::similar_names)]
#[test]
fn iv_5_three_shell_is_run_to_run_deterministic() {
    // Decision N determinism — same scene built twice; the
    // `materials()` cache is bit-equal across runs and `step.x_final`
    // is bit-equal for two independent `replay_step` calls. Cheap
    // second-pass guard against non-determinism leaking through any
    // of: SDF mesher (III-1 already gates this; IV-5 re-confirms
    // through the larger end-to-end pipeline), three-shell field
    // sampling, internal-pressure traction packing, Newton solve at
    // the static regime under FullVector loads.
    //
    // Runs at `h/2` (the canonical Phase 3 cell size) for ~7k-tet
    // bit-equality without paying the h/4 runtime cost twice.
    let (mesh_a, bc_a, initial_a, theta_a) =
        SoftScene::layered_silicone_sphere(three_shell_field(), CELL_SIZE_H2, PRESSURE)
            .expect("layered_silicone_sphere should mesh successfully at h/2");
    let (mesh_b, bc_b, initial_b, theta_b) =
        SoftScene::layered_silicone_sphere(three_shell_field(), CELL_SIZE_H2, PRESSURE)
            .expect("layered_silicone_sphere should mesh successfully at h/2");

    // Materials cache bit-equality.
    assert_eq!(mesh_a.materials().len(), mesh_b.materials().len());
    for (nh_a, nh_b) in mesh_a.materials().iter().zip(mesh_b.materials().iter()) {
        assert_neo_hookean_bit_equal(nh_a, nh_b);
    }

    // BC ordering bit-equality (loaded order drives theta packing).
    assert_eq!(
        bc_a.pinned_vertices, bc_b.pinned_vertices,
        "pinned_vertices ordering drift across runs — predicate non-determinism leak"
    );
    assert_eq!(
        bc_a.loaded_vertices, bc_b.loaded_vertices,
        "loaded_vertices ordering drift across runs — predicate non-determinism leak"
    );

    // Theta packing bit-equality.
    let theta_a_slice = theta_a.as_slice();
    let theta_b_slice = theta_b.as_slice();
    assert_eq!(theta_a_slice.len(), theta_b_slice.len());
    for (i, (&a_val, &b_val)) in theta_a_slice.iter().zip(theta_b_slice.iter()).enumerate() {
        assert_eq!(
            a_val.to_bits(),
            b_val.to_bits(),
            "theta[{i}] bit-drift across runs: {a_val:e} vs {b_val:e}",
        );
    }

    // Initial-state bit-equality (rest positions).
    assert_eq!(
        initial_a.x_prev.as_slice(),
        initial_b.x_prev.as_slice(),
        "initial x_prev bit-drift across runs"
    );

    // End-to-end solver-output bit-equality through `replay_step`.
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = 50;

    // Capture the first loaded vertex's id BEFORE moving bc_a into
    // the solver — used for the post-solve sanity check below.
    let first_loaded = bc_a.loaded_vertices[0].0 as usize;

    let solver_a: CpuTet4NHSolver<SdfMeshedTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh_a, NullContact, cfg, bc_a);
    let solver_b: CpuTet4NHSolver<SdfMeshedTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh_b, NullContact, cfg, bc_b);

    let step_a = solver_a.replay_step(&initial_a.x_prev, &initial_a.v_prev, &theta_a, cfg.dt);
    let step_b = solver_b.replay_step(&initial_b.x_prev, &initial_b.v_prev, &theta_b, cfg.dt);

    let x_final_a = step_a.x_final.as_slice();
    let x_final_b = step_b.x_final.as_slice();
    assert_eq!(x_final_a.len(), x_final_b.len());
    for (i, (&a_val, &b_val)) in x_final_a.iter().zip(x_final_b.iter()).enumerate() {
        assert_eq!(
            a_val.to_bits(),
            b_val.to_bits(),
            "x_final[{i}] bit-drift across runs: {a_val:e} vs {b_val:e}",
        );
    }

    // Sanity — the determinism gate is meaningful only if the scene
    // actually displaced. A degenerate static-equilibrium with zero
    // load would be trivially deterministic without exercising any
    // pipeline. Pick the first cavity-surface vertex's final position
    // and confirm it moved radially outward (cavity inflated).
    let final_pos = Vec3::new(
        x_final_a[3 * first_loaded],
        x_final_a[3 * first_loaded + 1],
        x_final_a[3 * first_loaded + 2],
    );
    let final_radius = final_pos.norm();
    let r_cavity = LAYERED_SPHERE_R_CAVITY;
    assert!(
        final_radius > r_cavity,
        "first cavity vertex should have inflated outward (final_radius = {final_radius:e}, \
         R_cavity = {r_cavity:e}); a non-displacing scene makes the determinism gate a \
         no-op",
    );
}
