//! F-bar volumetric-locking gate — the near-incompressible cure, measured on a
//! cantilever against the locking signature it is meant to remove.
//!
//! ## What locks, and what F-bar fixes
//!
//! A single-Gauss-point [Tet4 element](../src/element/tet4.rs) imposes one
//! volumetric constraint (`J ≈ 1`) per element. As `ν → 0.5` the volumetric
//! stiffness `λ` diverges (`λ/μ ≈ 49` at `ν = 0.49`, the Ecoflex 00-30 regime),
//! the per-element constraints over-constrain the 12-DOF element, and the
//! bending (isochoric) modes are suppressed — *volumetric locking*. The
//! mechanical signature (Part 2 Ch 05 00-locking.md): a cantilever's tip
//! deflection **collapses** as `ν → 0.5`, even though the physical deflection is
//! nearly `ν`-independent (`δ_EB ∝ 1/E`, and `E = 2μ(1+ν)` moves only ~6% across
//! the swept range).
//!
//! F-bar (`SolverConfig.fbar`) feeds the constitutive law the patch-modified
//! kinematic `F* = (J̄/J)^{1/3}·F` — replacing the per-element volumetric
//! constraint with a nodal-*patch*-averaged one — and removes the collapse.
//!
//! ## The offset-free metric
//!
//! This gate does NOT assert the tip deflection matches Euler-Bernoulli
//! absolutely: the `L/H = 5` beam at this coarse refinement carries a large,
//! `ν`-independent discretization *factor* (constant-strain Tet4 under-resolves
//! bending — it scales the deflection down by a roughly constant multiple), and
//! simple F-bar on the *coupled* Neo-Hookean energy modifies the response
//! beyond the strictly-locked regime. Because both are **multiplicative and
//! `ν`-independent**, they cancel in the **ratio** `δ(ν_hi)/δ(ν_lo)` — a purely
//! additive offset would NOT cancel, but a factor does — leaving only the
//! `ν`-sensitivity, i.e. the locking itself. Plain Tet4's ratio collapses far
//! below the physical value; F-bar's stays near it. That differential is the
//! locking cure, cleanly. (Absolute-accuracy validation of the cured `ν=0.49`
//! response — F-bar → EB under mesh refinement, or the #676 contact gate at
//! `ν=0.49` — is a separate rung; this gate proves the cure, not the accuracy.)

// Saint-Venant-averaged tip deflection (`Σ z / N`) and the per-vertex load
// split (`F / N`) cast the loaded-vertex count to `f64` — the canonical FEM
// end-load idiom, not a precision-sensitive numeric path.
#![allow(clippy::cast_precision_loss)]

use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, HandBuiltTetMesh, LoadAxis,
    MaterialField, Mesh, NullContact, Solver, SolverConfig, Tet4, VertexId,
    pick_vertices_by_predicate,
};

const MU: f64 = 1.0e5;
const LENGTH: f64 = 0.5;
const BREADTH: f64 = 0.1;
const HEIGHT: f64 = 0.1;
/// Small tip load — the locking signature is a small-strain (near-linear)
/// phenomenon; a light load keeps the beam well inside the Neo-Hookean regime.
const TIP_FORCE_TOTAL: f64 = 1.0;
/// Large `dt` → the inertial term `M/dt²` collapses and one backward-Euler
/// `replay_step` from rest converges to the static equilibrium (the
/// `bonded_bilayer_beam.rs` static-regime idiom).
const STATIC_DT: f64 = 1.0;
/// Low Poisson ratio — Tet4 is (nearly) unlocked here; the reference rung.
const NU_LO: f64 = 0.30;
/// Ecoflex 00-30's real Poisson ratio — deep in the locked regime for plain
/// Tet4, the whole reason this rung exists.
const NU_HI: f64 = 0.49;

/// `λ` from `(μ, ν)`: `λ = 2μν/(1−2ν)`.
fn lambda_from_nu(nu: f64) -> f64 {
    2.0 * MU * nu / (1.0 - 2.0 * nu)
}

/// Single static `replay_step` on a uniform `(8,2,2)` cantilever at Poisson
/// ratio `nu`, with F-bar on/off. Returns the Saint-Venant-averaged tip
/// `z`-deflection (mean over the `x = LENGTH` face).
fn tip_deflection(nu: f64, fbar: bool) -> f64 {
    let field = MaterialField::uniform(MU, lambda_from_nu(nu));
    let mesh = HandBuiltTetMesh::cantilever_bilayer_beam(8, 2, 2, LENGTH, BREADTH, HEIGHT, &field);

    let pinned: Vec<VertexId> = pick_vertices_by_predicate(&mesh, |p| p.x.abs() < 1e-9);
    let loaded: Vec<VertexId> = pick_vertices_by_predicate(&mesh, |p| (p.x - LENGTH).abs() < 1e-9);

    let positions = mesh.positions();
    let n_dof = 3 * positions.len();
    let mut x_prev_flat = vec![0.0; n_dof];
    for (v, pos) in positions.iter().enumerate() {
        x_prev_flat[3 * v] = pos.x;
        x_prev_flat[3 * v + 1] = pos.y;
        x_prev_flat[3 * v + 2] = pos.z;
    }
    let loaded_rest_z: Vec<(VertexId, f64)> = loaded
        .iter()
        .map(|&v| (v, positions[v as usize].z))
        .collect();

    let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
    let v_prev = Tensor::zeros(&[n_dof]);
    let theta = Tensor::from_slice(&[TIP_FORCE_TOTAL / loaded.len() as f64], &[1]);

    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: Vec::new(),
        loaded_vertices: loaded.iter().map(|&v| (v, LoadAxis::AxisZ)).collect(),
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    // Near-incompressible F-bar Newton needs headroom — the ν=0.49 solve is
    // genuinely stiff (~80 iters observed); the plain / low-ν solves are far
    // quicker. Forward-only path (`replay_step`), so F-bar is supported.
    cfg.max_newton_iter = 150;
    cfg.fbar = fbar;

    let solver: CpuTet4NHSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);
    let step = solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt);

    let sum: f64 = loaded_rest_z
        .iter()
        .map(|&(v, rest_z)| step.x_final[3 * v as usize + 2] - rest_z)
        .sum();
    sum / loaded_rest_z.len() as f64
}

// Release-only gate: the ν=0.49 F-bar Newton solve is stiff (~80 iters over a
// denser F-bar 2-ring factor); the four solves run in seconds release-mode and
// far slower in debug. `#[cfg_attr(debug_assertions, ignore)]` skips it in the
// default-profile CI tests-debug tier while exercising it under `cargo test
// --release` (developer pre-push). Mirrors `bonded_layer_indentation.rs`.
#[cfg_attr(
    debug_assertions,
    ignore = "release-only — near-incompressible F-bar cantilever sweep (seconds \
              release, much slower debug); rerun with `cargo test --release`"
)]
#[test]
fn fbar_cures_cantilever_volumetric_locking() {
    let plain_lo = tip_deflection(NU_LO, false);
    let plain_hi = tip_deflection(NU_HI, false);
    let fbar_lo = tip_deflection(NU_LO, true);
    let fbar_hi = tip_deflection(NU_HI, true);

    // Physical `ν`-sensitivity: EB deflection ∝ 1/E = 1/[2μ(1+ν)], so the ideal
    // ratio is (1+ν_lo)/(1+ν_hi) — the beam should barely stiffen with ν.
    let physical_ratio = (1.0 + NU_LO) / (1.0 + NU_HI); // ≈ 0.872
    let plain_ratio = plain_hi / plain_lo;
    let fbar_ratio = fbar_hi / fbar_lo;
    let recovery = fbar_hi / plain_hi;

    eprintln!("=== F-bar cantilever locking gate (8,2,2) ===");
    eprintln!("physical δ(0.49)/δ(0.30) ratio ≈ {physical_ratio:.3}");
    eprintln!("plain  Tet4: δ_lo={plain_lo:.4e} δ_hi={plain_hi:.4e}  ratio={plain_ratio:.3}");
    eprintln!("Tet4+Fbar:   δ_lo={fbar_lo:.4e} δ_hi={fbar_hi:.4e}  ratio={fbar_ratio:.3}");
    eprintln!("recovery δ_fbar(0.49)/δ_plain(0.49) = {recovery:.2}×");

    // 1. Plain Tet4 LOCKS: its ν-ratio collapses far below the physical ~0.87
    //    (the tip deflection craters as ν→0.5). Measured ≈ 0.23.
    assert!(
        plain_ratio < 0.45,
        "plain Tet4 should show volumetric locking (ν-ratio {plain_ratio:.3} collapsed \
         below the physical {physical_ratio:.3}); got a ratio that did not collapse"
    );

    // 2. F-bar REMOVES the collapse: its ν-ratio stays near physical, and is far
    //    above plain Tet4's. Measured ≈ 0.71.
    assert!(
        fbar_ratio > 0.60,
        "F-bar should remove the locking collapse (ν-ratio {fbar_ratio:.3} should stay \
         near the physical {physical_ratio:.3})"
    );
    assert!(
        fbar_ratio > 2.0 * plain_ratio,
        "F-bar ν-ratio {fbar_ratio:.3} should dominate plain Tet4's locked {plain_ratio:.3}"
    );

    // 3. F-bar recovers substantial real deflection at ν=0.49 (the locked plain
    //    solve reaches only a fraction of it). Measured ≈ 4.2×.
    assert!(
        recovery > 3.0,
        "F-bar should recover the deflection plain Tet4 loses to locking at ν=0.49 \
         (got {recovery:.2}×, expected > 3×)"
    );
}
