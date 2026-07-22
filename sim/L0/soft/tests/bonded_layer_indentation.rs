//! Bonded-layer spherical indentation: soft↔rigid CONTACT force law vs a
//! geometry-matched, independently-FEA-validated analytic answer.
//!
//! A rigid sphere is pressed (displacement-controlled, via IPC log-barrier
//! contact) into a soft elastic block that is BONDED to a rigid substrate
//! (pinned bottom face) — i.e. an elastic *layer* of finite thickness `h` on
//! a rigid base. The measured reaction force `F_FEM(δ)` is compared to the
//! closed-form **bonded bottom-effect correction** to Hertz.
//!
//! ## Why this gate is load-bearing
//!
//! [`hertz_sphere_plane`](hertz_sphere_plane) validates only the contact-patch
//! *radius* under penalty contact — penalty's compliance band structurally
//! cannot reach the rigid-limit force-vs-depth, so it never asserts the contact
//! *force* law. This fixture closes that gap: with IPC (non-penetrating
//! log-barrier) contact and displacement control, it asserts the reaction
//! FORCE against a known analytic law — the exact physics the differentiable
//! body↔device co-design loop differentiates through.
//!
//! Crucially the oracle is chosen to **match the system under test's geometry
//! exactly**. Hertz's half-space is unreachable by a finite FEM sample without
//! an intractable domain (a uniform mesh cannot be both patch-resolved and
//! half-space-large). The bonded finite block, however, *is* a bonded elastic
//! layer on a rigid base — for which Dimitriadis/Garcia give a closed-form
//! finite-thickness correction validated against independent FEA to <1%. So we
//! validate against the law our geometry actually satisfies, not an idealisation
//! it does not.
//!
//! ## The analytic oracle (bonded bottom-effect correction)
//!
//! For a paraboloidal (⇔ small-strain spherical) indenter of radius `R` pressed
//! by indentation `δ` into an elastic layer of thickness `h` bonded to a rigid
//! substrate:
//!
//! ```text
//! F_layer = F_Hertz_halfspace · Δ_bonded(χ),   χ = √(R·δ) / h
//! F_Hertz_halfspace = (4/3) · E* · √R · δ^{3/2},   E* = E / (1 − ν²)
//! ```
//!
//! `χ = √(Rδ)/h` is the Hertz contact radius `a = √(Rδ)` divided by the layer
//! thickness — the sole nondimensional group governing the bottom effect. As
//! `χ → 0` (thick layer) `Δ → 1` (half-space recovered); as `χ` grows the
//! bonded base confines the deformation and stiffens the response (`Δ > 1`).
//!
//! `Δ_bonded` is the Garcia form (Garcia et al.; the arbitrary-Poisson-ratio
//! coefficients derived in Dal Fabbro, Holuigue, Chighizola & Podestà,
//! *"Validation of contact mechanics models for AFM via FEA and nanoindentation"*,
//! arXiv:2406.17157, Note S2 Eq. S4):
//!
//! ```text
//! Δ_G(ν, χ) = 1 + A·χ + B·χ² + C·χ³ + D·χ⁴  (+ E·χ⁵)
//! α₀ = −(1.2876 − 1.4678ν + 1.3442ν²) / (1 − ν)      (< 0)
//! β₀ =  (0.6387 − 1.0277ν + 1.5164ν²) / (1 − ν)
//! A = |2α₀/π|   (analytically exact)
//! B = 301π α₀² / 2000
//! C = −π (31α₀³/255 + 106β₀/491)
//! D =  π (24α₀⁴/245 + 40β₀α₀/97)
//! E = −π (α₀⁵/22  + 2β₀α₀²/9)
//! ```
//!
//! **Sign note on `A`.** The source prints `A = 2α₀/π`; with its own `α₀ < 0`
//! that is negative, but the physical leading coefficient is positive (the
//! bonded layer stiffens). The magnitude is used here — verified against the
//! source's own published values below.
//!
//! **Verification of the coefficient formula (done before committing this
//! gate).** Evaluated at the source's reference ratios, [`delta_garcia`]
//! reproduces the published Garcia coefficients to ≲ 1.5 % (A, B, C within
//! ~1 %; the χ⁴ coefficient `D` is the loosest at ~1.2 % at ν = 0.49):
//! `ν = 0.50 → A,B,C,D = 1.133, 1.497, 1.469, 0.755` and
//! `ν = 0.49 → 1.112, 1.444, 1.374, 0.645`. This gate runs at `ν = 0.4` (Tet4
//! avoids volumetric locking there — see below), giving
//! `Δ_G(0.4) = 1 + 0.9714χ + 1.1009χ² + 0.8254χ³ + 0.1192χ⁴ − 0.0926χ⁵`
//! (the χ⁵ term contributes ~0.15 % at χ = 0.5).
//!
//! ## Why the residual is a *floor*, not a match — the Tet4 ceiling
//!
//! `F_FEM` reproduces the *shape* `Δ(χ)` (see the χ-sweep asserts) but sits
//! `~5–13 %` above it. This residual is a genuine, mesh-*converged* floor, not
//! under-resolution — asserted by the committed 3-level cell-convergence
//! sub-check: at χ = 0.35 the ratio decrements shrink (Cauchy) across
//! `a/cell = 2 → 3 → 4` (`|Δ| ≈ 0.037 → 0.010`) and the geometric-tail
//! extrapolated limit (`≈ 1.09`) sits clearly above 1, i.e. the `RATIO` does
//! NOT approach 1 under refinement. It is attributable to **Tet4 element
//! over-stiffness** on the curved contact patch plus a small IPC `d̂`-band
//! contribution — a known element-order limitation (the same reason
//! [`hertz_sphere_plane`] and the Phase-4 gates defer near-incompressible
//! accuracy to a future Tet10 + F-bar element), NOT a defect in the contact
//! force law. The floor grows mildly with χ (thinner layers develop steeper
//! through-thickness gradients that linear Tet4 resolves poorly). This gate
//! therefore asserts the ratio lies in a **documented band**, not that it
//! equals 1; Tet10 + F-bar (and `d̂ → 0`) are the rungs that would tighten it.
//!
//! ## Regime (kept inside the oracle's validity, and mesh-resolvable)
//!
//! - `δ/R = 0.05` — small-strain, so the spherical indenter ≈ paraboloid (the
//!   large-indentation / sphere-vs-paraboloid correction is negligible) and
//!   Neo-Hookean ≈ linear elasticity (the oracle is linear-elastic).
//! - `ν = 0.4` (`λ = 4μ`) — Tet4 volumetric locking precludent `ν → 0.5`
//!   (Phase-4 IV-3/IV-5 precedent); the ν-dependent oracle is evaluated at this
//!   same `ν`, so oracle and SUT match.
//! - `χ ∈ {0.20, 0.35, 0.50}` swept by varying `h` at fixed `R, δ` — the layer
//!   is *thin* (h ~ a), so it meshes cheaply (a few cells through thickness),
//!   sidestepping the half-space resolution wall entirely.
//! - Lateral extent `= 8a`, cell `= a/3` (near-converged: `a/cell = 3` sits
//!   within ~1 % of `a/cell = 4`). The bonded correction assumes a laterally
//!   unbounded layer; `8a` keeps the free side boundaries far from the patch.
//!
//! ## Displacement control (the IPC-feasibility ramp)
//!
//! IPC's barrier requires a strictly feasible (non-penetrating) start, so the
//! rigid sphere is introduced with its south pole `1.2·d̂` ABOVE the surface
//! (barrier inactive) and lowered in `0.3·d̂` increments (warm-started from the
//! previous converged state, `v_prev = 0`). A larger step from an
//! already-active start lands the first contact deep in the thin barrier where
//! the barrier Hessian `b'' ≈ κ·122` makes the condensed tangent near-singular
//! and the Armijo line-search stalls. There are no loaded vertices — the fixed
//! indenter geometry *is* the load — so `theta` is the empty tensor.
//!
//! ## Asserted shape
//!
//! - **χ-sweep** (`a/cell = 3`): `F_FEM/Hertz` strictly rises with χ (the
//!   contact responds to the finite thickness); each
//!   `RATIO = F_FEM / (Δ_G(0.4)·Hertz)` lies in `[1.00, 1.20]` (the Tet4+band
//!   floor); Newton converges each increment.
//! - **Oracle-explains-the-variation** (negative control): dividing by `Δ_G`
//!   collapses the χ-variation — `F_FEM/Hertz` spans a factor `> 1.4` across the
//!   sweep, yet the `RATIO` spans a factor `< 1.12`. A model that ignored the
//!   finite thickness could not both track the ~60 % force rise and leave the
//!   ratio nearly flat.
//! - **Cell convergence** (proves *floor*, not under-resolution): at `χ = 0.35`,
//!   across `a/cell = 2 → 3 → 4` the ratio decreases monotonically, its
//!   decrements shrink (Cauchy: `|Δ_fine| < |Δ_coarse|`), and the
//!   geometric-tail extrapolated limit stays `> 1.03` — mesh-converged to a
//!   floor above 1, not a coincidence of one resolution and not heading to 1.

#![allow(
    // Helpers `.expect(...)` on the meshing/among-tuple returns — mirrors
    // `hertz_sphere_plane.rs` / `concentric_lame_shells.rs` precedent.
    clippy::expect_used,
    // Analytic-comparison test with an inlined coefficient oracle + a
    // multi-point sweep + per-point + cross-point asserts legitimately exceeds
    // clippy's 100-line soft cap (same as `hertz_sphere_plane.rs`).
    clippy::too_many_lines
)]

use sim_ml_chassis::Tensor;
use sim_soft::readout::scene::pick_vertices_by_predicate;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, IpcRigidContact, MaterialField, Mesh,
    Solver, SolverConfig, SphereSdf, Tet4, TranslatedSdf, Vec3,
};

// ── Material (ν = 0.4 avoids Tet4 volumetric locking; λ = 4μ ⇒ ν = 0.4) ────
const MU: f64 = 1.0e5;
const LAMBDA: f64 = 4.0 * MU;

// ── Indentation geometry (small-strain: paraboloid ≈ sphere, NH ≈ linear) ──
/// Sphere radius (10 mm).
const RADIUS: f64 = 1.0e-2;
/// Indentation depth (0.5 mm ⇒ δ/R = 0.05, a = √(Rδ) = 2.236 mm).
const DELTA: f64 = 5.0e-4;

// ── Contact + solver ───────────────────────────────────────────────────────
/// IPC barrier stiffness (default) — held fixed across the sweep.
const KAPPA: f64 = 1.0e4;
/// IPC band as a fraction of δ. Thin (`d̂ ≪ δ`) so the band's contribution to
/// the floor is small; the ramp lands first contact shallow at `~0.9·d̂`.
const BAND_FRAC: f64 = 0.05;
/// Static-equilibrium time-step — collapses the inertial `M/dt²` term so each
/// increment is a pure static root-find (mirrors `hertz_sphere_plane.rs`).
const STATIC_DT: f64 = 1.0;
/// Newton iteration cap per increment.
const MAX_NEWTON_ITER: usize = 80;

// ── Domain (thin bonded layer; laterally wide vs the contact patch) ────────
/// Lateral extent in units of the Hertz contact radius `a`. The oracle assumes
/// a laterally unbounded layer; `8a` keeps free side boundaries far from the
/// patch (~7a of margin from the contact axis).
const LATERAL_FACTOR: f64 = 8.0;
/// Cell-to-`a` ratio for the primary sweep. `a/cell = 3` is near-converged
/// (within ~1 % of `a/cell = 4` at χ = 0.35).
const A_OVER_CELL: f64 = 3.0;

// ── Swept χ values (χ = √(Rδ)/h, varied by h) ──────────────────────────────
const CHI_SWEEP: [f64; 3] = [0.20, 0.35, 0.50];

// ── Asserted band / spreads (measured converged values: RATIO = 1.051 /
//    1.105 / 1.130; F/Hertz = 1.309 / 1.670 / 2.112) ─────────────────────────
const RATIO_LO: f64 = 1.00;
const RATIO_HI: f64 = 1.20;
const FHERTZ_SPREAD_MIN: f64 = 1.4; // measured 1.614
const RATIO_SPREAD_MAX: f64 = 1.12; // measured 1.076

// ── Material helpers ───────────────────────────────────────────────────────

fn nu() -> f64 {
    LAMBDA / (2.0 * (LAMBDA + MU))
}

/// Young's modulus from the Lamé pair: `E = μ(3λ + 2μ)/(λ + μ)`.
fn young() -> f64 {
    MU * 2.0f64.mul_add(MU, 3.0 * LAMBDA) / (LAMBDA + MU)
}

/// Hertz contact modulus `E* = E/(1 − ν²)`.
fn e_star() -> f64 {
    young() / 1.0f64.mul_add(-(nu() * nu()), 1.0)
}

/// Half-space Hertz force `F = (4/3)·E*·√R·δ^{3/2}` for a paraboloidal indenter.
fn hertz_halfspace(delta: f64) -> f64 {
    4.0 / 3.0 * e_star() * RADIUS.sqrt() * delta.powf(1.5)
}

/// Garcia bonded bottom-effect correction `Δ_G(ν, χ)` (arXiv:2406.17157 Note S2
/// Eq. S4). Verified against the source's published coefficients at ν = 0.5 /
/// 0.49 (see module docstring). `χ = √(Rδ)/h`.
fn delta_garcia(nu: f64, chi: f64) -> f64 {
    let a0 = -(1.3442f64.mul_add(nu * nu, 1.4678f64.mul_add(-nu, 1.2876))) / (1.0 - nu);
    let b0 = 1.5164f64.mul_add(nu * nu, 1.0277f64.mul_add(-nu, 0.6387)) / (1.0 - nu);
    let pi = std::f64::consts::PI;
    let ca = (2.0 * a0 / pi).abs(); // leading, analytically exact
    let cb = 301.0 * pi * a0 * a0 / 2000.0;
    let cc = -pi * (31.0 * a0.powi(3) / 255.0 + 106.0 * b0 / 491.0);
    let cd = pi * (24.0 * a0.powi(4) / 245.0 + 40.0 * b0 * a0 / 97.0);
    let ce = -pi * (a0.powi(5) / 22.0 + 2.0 * b0 * a0 * a0 / 9.0);
    // Horner in χ.
    ce.mul_add(chi, cd)
        .mul_add(chi, cc)
        .mul_add(chi, cb)
        .mul_add(chi, ca)
        .mul_add(chi, 1.0)
}

fn material_field() -> MaterialField {
    MaterialField::uniform(MU, LAMBDA)
}

// ── Scene ──────────────────────────────────────────────────────────────────

/// Thin soft layer `Lx × Ly × h`, uniform material, built as a non-cube box
/// (small `nz` ⇒ few cells through thickness). Bonded bottom (`z = 0`), free
/// top/sides.
fn layer(nx: usize, ny: usize, nz: usize, lx: f64, ly: f64, h: f64) -> HandBuiltTetMesh {
    HandBuiltTetMesh::cantilever_bilayer_beam(nx, ny, nz, lx, ly, h, &material_field())
}

/// Rigid sphere posed with its centre over the layer's top-centre at `z_center`.
fn indenter(lx: f64, ly: f64, z_center: f64) -> TranslatedSdf<SphereSdf> {
    TranslatedSdf {
        inner: SphereSdf { radius: RADIUS },
        offset: Vec3::new(lx / 2.0, ly / 2.0, z_center),
    }
}

/// One indentation run. Returns the reaction force `F_FEM` and diagnostics.
struct Indentation {
    /// Reaction force `|Σ force_on_soft.z|` at the final pose.
    f_fem: f64,
    /// Active contact-pair count at the final pose (sanity: `> 0`).
    n_active: usize,
    /// Max Newton iterations over the ramp.
    max_iters: usize,
    /// Max free-DOF residual over the ramp.
    max_res: f64,
}

/// Displacement-controlled IPC indentation. The sphere south pole starts a full
/// `1.2·d̂` above the top face (barrier inactive) and is lowered in `0.3·d̂`
/// increments to `δ` below the surface, warm-started each step (`v_prev = 0`).
/// See the module docstring's "Displacement control" section for why a gentler
/// start than the naive `0.4·d̂` is required.
fn run_indentation(nx: usize, ny: usize, nz: usize, lx: f64, ly: f64, h: f64) -> Indentation {
    let d_hat = BAND_FRAC * DELTA;
    let top_z = h;
    let z_start = top_z + RADIUS + 1.2 * d_hat;
    let z_end = top_z + RADIUS - DELTA;
    let step = 0.3 * d_hat;

    let mesh = layer(nx, ny, nz, lx, ly, h);
    let n_dof = 3 * mesh.n_vertices();
    let pins = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    assert!(
        !pins.is_empty(),
        "bonded bottom face has no vertices at z = 0"
    );

    let mut x_prev: Vec<f64> = {
        let mut x = vec![0.0_f64; n_dof];
        for (c, p) in x.chunks_exact_mut(3).zip(mesh.positions().iter()) {
            c[0] = p.x;
            c[1] = p.y;
            c[2] = p.z;
        }
        x
    };
    let v_prev = vec![0.0_f64; n_dof];

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = MAX_NEWTON_ITER;
    let empty_theta = Tensor::from_slice(&[], &[0]);

    let mut z = z_start;
    let mut max_iters = 0usize;
    let mut max_res = 0.0f64;
    loop {
        let target = if z - step <= z_end { z_end } else { z - step };
        z = target;

        let contact = IpcRigidContact::with_params(vec![indenter(lx, ly, z)], KAPPA, d_hat);
        let solver: CpuNewtonSolver<Tet4, HandBuiltTetMesh, IpcRigidContact> = CpuNewtonSolver::new(
            Tet4,
            layer(nx, ny, nz, lx, ly, h),
            contact,
            cfg,
            BoundaryConditions::new(pins.clone(), Vec::new()),
        );
        let out = solver.replay_step(
            &Tensor::from_slice(&x_prev, &[n_dof]),
            &Tensor::from_slice(&v_prev, &[n_dof]),
            &empty_theta,
            STATIC_DT,
        );
        max_iters = max_iters.max(out.iter_count);
        max_res = max_res.max(out.final_residual_norm);
        x_prev = out.x_final;

        if (z - z_end).abs() < 1e-12 {
            break;
        }
    }

    // Reaction force at the final pose (contact is not Clone — rebuild for readout).
    let readout_contact = IpcRigidContact::with_params(vec![indenter(lx, ly, z_end)], KAPPA, d_hat);
    let positions: Vec<Vec3> = x_prev
        .chunks_exact(3)
        .map(|c| Vec3::new(c[0], c[1], c[2]))
        .collect();
    let readout = readout_contact.per_pair_readout(&mesh, &positions);
    let f_fem = readout.iter().map(|r| r.force_on_soft.z).sum::<f64>().abs();

    Indentation {
        f_fem,
        n_active: readout.len(),
        max_iters,
        max_res,
    }
}

/// Mesh dimensions for a given χ and `a/cell`, holding the contact patch `a`
/// fixed. Returns `(nx = ny, nz, Lx = Ly, h)`. `nz` is forced even (the
/// bilayer-beam builder aligns a would-be interface at `z = h/2`).
// Cell counts are small positive integers well below `usize`/`f64`-mantissa
// limits — the `.round() as usize` casts are exact (same idiom as the Phase-4
// mesh builders).
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn dims_for(chi: f64, a_over_cell: f64) -> (usize, usize, f64, f64) {
    let a = (RADIUS * DELTA).sqrt();
    let cell = a / a_over_cell;
    let h = a / chi;
    let lateral = LATERAL_FACTOR * a;
    let even = |n: usize| if n.is_multiple_of(2) { n } else { n + 1 };
    let n_lat = even(((lateral / cell).round() as usize).max(2));
    let nz = even(((h / cell).round() as usize).max(2));
    (n_lat, nz, lateral, h)
}

// ── Test ─────────────────────────────────────────────────────────────────

// Release-only gate. IPC + the multi-increment displacement ramp over several
// meshes runs in minutes release-mode and `5-10×` slower in debug, over the CI
// 30-min budget. `#[cfg_attr(debug_assertions, ignore)]` skips it in
// default-profile `cargo test` (the CI tests-debug tier) while exercising it
// under `cargo test --release` (developer pre-push verification). Mirrors
// `hertz_sphere_plane.rs`.
#[cfg_attr(
    debug_assertions,
    ignore = "release-only — IPC bonded-layer indentation sweep (minutes release, \
              much slower debug); rerun with `cargo test --release` to include"
)]
#[test]
fn bonded_layer_indentation_matches_analytic_correction() {
    let f_hs = hertz_halfspace(DELTA);
    let a = (RADIUS * DELTA).sqrt();

    // ── χ-sweep at a/cell = 3 ────────────────────────────────────────────
    let mut fh_ratio = Vec::new(); // F_FEM / Hertz_halfspace
    let mut oracle_ratio = Vec::new(); // F_FEM / (Δ_G · Hertz)
    for &chi in &CHI_SWEEP {
        let (n_lat, nz, lateral, h) = dims_for(chi, A_OVER_CELL);
        let r = run_indentation(n_lat, n_lat, nz, lateral, lateral, h);
        let delta = delta_garcia(nu(), chi);
        let fh = r.f_fem / f_hs;
        let ratio = r.f_fem / (delta * f_hs);
        eprintln!(
            "χ = {chi:.2}  h = {h:.4} m  n_lat = {n_lat}  nz = {nz}  \
             F_FEM = {ff:.5} N  F/Hertz = {fh:.4}  Δ_G(0.4) = {delta:.4}  \
             RATIO = {ratio:.4}  (n_active = {na}, max_iters = {mi}, max_res = {mr:.2e})",
            ff = r.f_fem,
            na = r.n_active,
            mi = r.max_iters,
            mr = r.max_res,
        );
        assert!(
            r.max_iters < MAX_NEWTON_ITER,
            "Newton hit the {MAX_NEWTON_ITER}-iter cap at χ = {chi} — solver/contact regime \
             regression",
        );
        assert!(
            r.n_active > 0,
            "no active contact pairs at χ = {chi} — sphere never engaged"
        );
        assert!(
            (RATIO_LO..=RATIO_HI).contains(&ratio),
            "RATIO = {ratio:.4} at χ = {chi} outside the documented Tet4+band floor \
             [{RATIO_LO}, {RATIO_HI}] — F_FEM = {ff:.5} N, Δ_G·Hertz = {orc:.5} N. Either the \
             contact force law regressed or the floor moved; investigate before widening.",
            ff = r.f_fem,
            orc = delta * f_hs,
        );
        fh_ratio.push(fh);
        oracle_ratio.push(ratio);
    }

    // ── F_FEM/Hertz strictly rises with χ (contact responds to thickness) ──
    for w in fh_ratio.windows(2) {
        assert!(
            w[1] > w[0],
            "F_FEM/Hertz not increasing with χ ({:.4} → {:.4}) — the FEM is not registering the \
             bonded-layer bottom effect",
            w[0],
            w[1],
        );
    }

    // ── Oracle explains the variation (negative control) ──────────────────
    // F_FEM/Hertz spans a large factor across the sweep, yet dividing by Δ_G
    // collapses it to a narrow band — the finite-thickness correction is what
    // the FEM is tracking, not a coincidental constant offset.
    let fh_spread = fh_ratio.last().expect("sweep non-empty") / fh_ratio[0];
    let ratio_max = oracle_ratio.iter().copied().fold(f64::MIN, f64::max);
    let ratio_min = oracle_ratio.iter().copied().fold(f64::MAX, f64::min);
    let ratio_spread = ratio_max / ratio_min;
    eprintln!(
        "F/Hertz spread = {fh_spread:.4} (assert > {FHERTZ_SPREAD_MIN}); \
         RATIO spread = {ratio_spread:.4} (assert < {RATIO_SPREAD_MAX})",
    );
    assert!(
        fh_spread > FHERTZ_SPREAD_MIN,
        "F_FEM/Hertz spread {fh_spread:.4} ≤ {FHERTZ_SPREAD_MIN} — the χ sweep does not exercise a \
         large enough bottom-effect range to be discriminating",
    );
    assert!(
        ratio_spread < RATIO_SPREAD_MAX,
        "RATIO spread {ratio_spread:.4} ≥ {RATIO_SPREAD_MAX} — dividing by Δ_G did NOT collapse the \
         χ-variation, so the analytic correction is not the law the FEM follows",
    );

    // ── Cell convergence at χ = 0.35 (a/cell 2 → 3 → 4: Cauchy + floored) ──
    // Three refinement levels are needed to distinguish a genuine floor above 1
    // (Tet4 element bias) from slow convergence toward 1 (mere under-resolution):
    // two monotone-decreasing points cannot tell them apart. a/cell = 3 is
    // reused from the sweep; add a/cell = 2 (coarse) and a/cell = 4 (fine).
    let chi_c = 0.35;
    let delta_c = delta_garcia(nu(), chi_c);
    let ratio_at = |div: f64| {
        let (nl, nz, lat, h) = dims_for(chi_c, div);
        run_indentation(nl, nl, nz, lat, lat, h).f_fem / (delta_c * f_hs)
    };
    let ratio_c2 = ratio_at(2.0);
    let ratio_c3 = oracle_ratio[1]; // a/cell = 3, already computed in the sweep
    let ratio_c4 = ratio_at(4.0);
    let step_coarse = (ratio_c2 - ratio_c3).abs();
    let step_fine = (ratio_c3 - ratio_c4).abs();
    // Geometric-tail extrapolation of the converged limit from the last two
    // decrements: limit ≈ ratio_c4 − step_fine · r/(1 − r), r = step_fine/step_coarse.
    let r = step_fine / step_coarse;
    let extrap_limit = ratio_c4 - step_fine * r / (1.0 - r);
    eprintln!(
        "cell convergence at χ = {chi_c}: RATIO(a/cell 2,3,4) = {ratio_c2:.4}, {ratio_c3:.4}, \
         {ratio_c4:.4}; steps |Δ| = {step_coarse:.4} → {step_fine:.4} (r = {r:.3}); \
         extrapolated floor = {extrap_limit:.4}",
    );
    // Monotone-downward: refinement resolves out the coarse-patch bias.
    assert!(
        ratio_c4 < ratio_c3 && ratio_c3 < ratio_c2,
        "refinement did not monotonically reduce the ratio ({ratio_c2:.4} → {ratio_c3:.4} → \
         {ratio_c4:.4}) — the coarse-patch bias is not being resolved out as expected",
    );
    // Cauchy: the decrements shrink ⇒ the sequence converges (not drifting).
    assert!(
        step_fine < step_coarse,
        "cell-refinement decrements not shrinking (|Δ_coarse| = {step_coarse:.4}, \
         |Δ_fine| = {step_fine:.4}) at χ = {chi_c} — the ratio is not converging in cell size, so \
         the reported floor would be resolution-dependent",
    );
    // The converged limit sits clearly ABOVE 1 ⇒ the residual is a genuine
    // (Tet4 + band) floor, NOT under-resolution that would vanish (limit → 1).
    assert!(
        extrap_limit > 1.03,
        "extrapolated cell-converged floor {extrap_limit:.4} ≤ 1.03 — the residual is heading to 1 \
         under refinement, i.e. it is under-resolution, NOT the claimed mesh-converged Tet4 floor. \
         Re-examine the floor characterization before trusting the [{RATIO_LO}, {RATIO_HI}] band.",
    );

    // Contact-radius / small-strain sanity (documentary): a = √(Rδ).
    eprintln!(
        "Hertz contact radius a = {a:.4e} m, δ/R = {:.3}",
        DELTA / RADIUS
    );
}
