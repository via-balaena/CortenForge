//! single-tet-stretch — `SkeletonSolver` end-to-end on `SoftScene::one_tet_cube`.
//!
//! Walking-skeleton uniaxial-traction step on the canonical 1-tet cube
//! per spec §2: decimeter-edge tet, `(μ, λ) = (1e5, 4e5)` Pa,
//! `ρ = 1030` kg/m³, `Δt = 1e-2` s, `θ = 10` N along `+ẑ` on `v_3`.
//! `v_0..v_2` are pinned (full Dirichlet); `v_3` is the loaded vertex
//! under `LoadAxis::AxisZ`. This is the **first FEM-running example**
//! in the sim-soft examples arc — rows 1+2+3 exercised the SDF +
//! meshing pipeline only; this row is `Solver::step` end-to-end through
//! `CpuTet4NHSolver<SingleTetMesh>` (= `SkeletonSolver`) with backward
//! Euler + Newton + Armijo to the Stage-1 R-1 convergence bound.
//!
//! Per `feedback_math_pass_first_handauthored`, a clean `cargo run
//! --release` exit-0 IS the correctness signal — every claim sits
//! behind an `assert_*` in a `verify_*` anchor group below. Output
//! artifact is `out/force_stretch.json` (no spatial artifact, no
//! cf-view rendering — single-tet has trivial topology and the
//! force-stretch readout is the inspection signal per inventory Q4
//! row 4).
//!
//! Anchor groups (all assertions exit-0 on success):
//!
//! - **Convergence** — Newton converges within `cfg.max_newton_iter`
//!   and `final_residual_norm < cfg.tol`.
//! - **Iter count** — `iter_count` exact-pin per the row-3 banked
//!   pattern: `step.x_final` is bit-stable on this dense small-FEM
//!   path (IV-1 contract), so the iteration count itself is also
//!   deterministic on a fixed toolchain — pin it exact for the
//!   tightest regression net.
//! - **State shape** — `step.x_final.len() == 12` (4 vertices × 3 DOFs).
//! - **Pinned-DOF Dirichlet** — every DOF of `v_0..v_2` (indices 0..9)
//!   sits within `1e-14` of its rest position (exact equality up to
//!   the 64-bit-float normalize-on-store FP noise).
//! - **Loaded-DOF sign** — `v_3.z` displacement strictly positive
//!   (force is `+ẑ`).
//! - **Loaded-DOF axis alignment** — `v_3.x` and `v_3.y` displacements
//!   below `1e-5` m. At F=I the NH constitutive tangent is diagonal
//!   under axis-aligned reference geometry; only `O(strain² · dz)` ~
//!   `1e-7` m off-axis displacement is induced by NH geometric
//!   nonlinearity at the ~1% strain regime here.
//! - **Dimensional band** — `v_3.z` displacement in `[5e-4, 1.5e-3]`
//!   m. Linear-estimate condensed stiffness `A_33 ≈ diag(2.1e3, 2.1e3,
//!   1.04e4)` N/m gives `δz ≈ θ / A_33,zz ≈ 9.6e-4` m at `θ = 10` N
//!   (~1% engineering strain), within the band by ~30% NH-nonlinearity
//!   margin on each side. See `solver_convergence.rs` module docstring
//!   for the dimensional analysis.
//! - **Bit-equal pre-Phase-4 reference** — every entry of `x_final`
//!   matches the IV-1 captured bit pattern at `c3729d4a` on rustc
//!   1.95.0 macOS arm64. Same re-capture protocol as IV-1: do NOT
//!   re-bake without ruling out a real toolchain delta and a real
//!   regression. See `invariant_iv_1_uniform_passthrough.rs` module
//!   docstring's "Two-tier contract" + "Failure-mode protocol"
//!   sections — the dense small-FEM bit-equality contract holds across
//!   `(macOS arm64, Linux x86_64)` and across rustc minor versions
//!   precisely because the 12-DOF assembly path is scalar-equivalent
//!   IEEE-754.

// `to_bits()` comparisons on f64 are the entire point of the IV-1 bit-
// equal anchor group — `clippy::float_cmp` flags this pattern.
// Mirrors `invariant_iv_1_uniform_passthrough.rs` line 92.
#![allow(clippy::float_cmp)]

use std::path::Path;

use anyhow::{Context, Result};
use serde_json::json;
use sim_ml_chassis::{Tape, Tensor};
use sim_soft::{
    CpuNewtonSolver, CpuTape, NewtonStep, NullContact, SceneInitial, SkeletonSolver, SoftScene,
    Solver, SolverConfig, Tet4,
};

// =============================================================================
// Constants
// =============================================================================

/// Stage-1 traction magnitude (N), `+ẑ` on `v_3`. Mirrors
/// `solver_convergence::stage_1_traction_converges` and IV-1's `THETA`
/// in `invariant_iv_1_uniform_passthrough.rs:119`.
const THETA: f64 = 10.0;

/// Lamé first parameter (Pa) — uniform Ecoflex-class material per
/// `SoftScene::one_tet_cube`'s
/// `MaterialField::uniform(1.0e5, 4.0e5)`. Surfaced here for the JSON
/// scene record; the actual mesh material is selected inside
/// `SoftScene::one_tet_cube`.
const MU_PA: f64 = 1.0e5;

/// Lamé second parameter (Pa) — see [`MU_PA`].
const LAMBDA_PA: f64 = 4.0e5;

/// Reference-configuration mass density (kg/m³) — `SolverConfig::skeleton`'s
/// silicone-class default per `solver/backward_euler.rs:98`.
const DENSITY_KG_M3: f64 = 1030.0;

/// Total DOF count (4 vertices × 3 components).
const N_DOF: usize = 12;

/// Number of fully-pinned DOFs — `v_0..v_2`'s xyz, indices `0..9` of
/// `x_final`. The first DOF NOT pinned is index `9 = N_PINNED_DOFS`,
/// which is `v_3.x` (= [`V3_X_DOF`]). Used as the upper bound of
/// [`verify_pinned_unmoved`]'s iterator.
const N_PINNED_DOFS: usize = 9;

/// `v_3`'s z-DOF index in the vertex-major + xyz-inner layout
/// (`v_3` = vertex 3, z is component 2 → index `3·3 + 2 = 11`).
const V3_Z_DOF: usize = 11;

/// `v_3`'s x-DOF index (`3·3 + 0 = 9`).
const V3_X_DOF: usize = 9;

/// `v_3`'s y-DOF index (`3·3 + 1 = 10`).
const V3_Y_DOF: usize = 10;

/// Pinned-DOF Dirichlet bound (m) — per
/// `solver_convergence.rs:113`'s
/// `< 1e-14` tolerance on the canonical scene. f64 round-to-nearest
/// noise on the rest positions sits at the last bit; `1e-14` admits it
/// while catching any real Dirichlet-pin regression.
const PINNED_DOF_BOUND_M: f64 = 1.0e-14;

/// Off-axis displacement bound (m) on `v_3.x` / `v_3.y`. F=I isotropic-
/// tangent claim per `solver_convergence.rs:88-94`. NH geometric
/// nonlinearity at ~1% strain induces at most `O(strain² · dz)`
/// ≈ `1e-7` m, two orders below this bound.
const DXY_BOUND_M: f64 = 1.0e-5;

/// Lower bound on `v_3.z` displacement (m). NH-nonlinearity-margined
/// dimensional band per `solver_convergence.rs:99`.
const DZ_LOWER_BOUND_M: f64 = 5.0e-4;

/// Upper bound on `v_3.z` displacement (m). See [`DZ_LOWER_BOUND_M`].
const DZ_UPPER_BOUND_M: f64 = 1.5e-3;

/// Linear-estimate dimensional-analysis prediction for `v_3.z`
/// displacement (m). Condensed stiffness at rest:
///
/// ```text
/// A_33 = (m / Δt²)·I + (V/L²) · diag(μ, μ, 2μ + λ)
///      ≈ 430·I + diag(1.67e3, 1.67e3, 1e4)
///      = diag(2.1e3, 2.1e3, 1.04e4)  N/m
/// δz   ≈ θ / A_33,zz ≈ 10 / 1.04e4 ≈ 9.6e-4  m
/// ```
///
/// See `solver_convergence.rs:14-26` for the derivation.
const ANALYTIC_LINEAR_ESTIMATE_M: f64 = 9.6e-4;

/// Newton iteration count exact-pin. Scope §3 R-1 predicts 3-5 iter
/// from rest at Stage-1 θ; on this dense 12-DOF assembly path the
/// iteration count is bit-stable on a fixed toolchain (companion to
/// the IV-1 [`ONE_TET_X_FINAL_BITS`] bit-equal contract — same
/// toolchain, same per-iter Newton residuals, same convergence
/// trigger). Row-3 banked pattern: pin every deterministic count exact.
const N_NEWTON_ITER_EXACT: usize = 3;

// -----------------------------------------------------------------------------
// IV-1 captured pre-Phase-4 reference bit patterns
// -----------------------------------------------------------------------------
//
// 1-tet skeleton `x_final` — 4 vertices × 3 DOFs in vertex-major + xyz-
// inner layout. Lifted verbatim from
// `sim/L0/soft/tests/invariant_iv_1_uniform_passthrough.rs:138-151`.
// IV-1's capture provenance:
//
//   "Bit-patterns were captured by running the canonical scene
//    fixtures on c3729d4a with `cargo test --release -p sim-soft --
//    --nocapture` on rustc 1.95.0 (2026-04-14) on macOS arm64, on
//    2026-04-27."
//
// IV-1 also locks in the **two-tier contract**: dense 12-DOF assembly
// (this path) is bit-equal across rustc/LLVM minor versions AND across
// `(macOS arm64, Linux x86_64)` SIMD architectures, because the
// `nalgebra::Matrix3` arithmetic compiles to scalar-equivalent
// IEEE-754 ops on every supported target. The sparse-faer path at
// scale gets relative-tol; this path gets bit-equal.
//
// **Failure-mode protocol** (mirrors IV-1's): if this assert fails,
// do NOT re-bake. Diagnose in this order:
//   1. Rule out toolchain drift (rustc / LLVM / libm minor version
//      delta vs rustc 1.95.0 capture).
//   2. If same toolchain, real regression — identify which sim-soft
//      commit altered the canonical-scene numerics.
//   3. NEVER re-bake the reference values to make the test green.
//
// Spurious re-capture hollows the contract to a tautology. Toolchain-
// environment drift is the only sanctioned re-capture trigger.

/// Frozen reference bit pattern for `x_final` on the canonical
/// 1-tet skeleton step at `θ = 10` N. Captured at sim-soft `c3729d4a`
/// per IV-1; see the comment block above.
const ONE_TET_X_FINAL_BITS: [u64; N_DOF] = [
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_9999_9999_999a,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_9999_9999_999a,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_d91e_b89f_fc81,
];

// =============================================================================
// Builder
// =============================================================================

/// Build the canonical scene + skeleton solver, run one `Solver::step`
/// at `θ = THETA` along `+ẑ` on `v_3`, return the step result + the
/// initial scene state + the solver config (the latter two threaded
/// through to verify functions and the JSON / stdout summaries).
fn run_skeleton_step() -> (NewtonStep<CpuTape>, SceneInitial, SolverConfig) {
    let cfg = SolverConfig::skeleton();
    let (mesh, bc, initial) = SoftScene::one_tet_cube();
    let mut solver: SkeletonSolver = CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);

    let mut tape = Tape::new();
    let theta_var = tape.param_tensor(Tensor::from_slice(&[THETA], &[1]));

    let step = solver.step(
        &mut tape,
        &initial.x_prev,
        &initial.v_prev,
        theta_var,
        cfg.dt,
    );
    (step, initial, cfg)
}

// =============================================================================
// verify_newton_convergence
// =============================================================================

/// Newton converged within budget AND final residual sits below the
/// configured tolerance. Anchors `Solver::step`'s convergence contract
/// (scope §3 R-1: 3-5 iter from rest at Stage-1 θ; loop exits via
/// convergence return, so `iter_count < max_newton_iter` always —
/// equality is unreachable on a converged path).
fn verify_newton_convergence(step: &NewtonStep<CpuTape>, cfg: &SolverConfig) {
    assert!(
        step.iter_count < cfg.max_newton_iter,
        "Newton did not converge within budget: iter_count = {} >= max_newton_iter = {}",
        step.iter_count,
        cfg.max_newton_iter,
    );
    assert!(
        step.final_residual_norm < cfg.tol,
        "Final residual norm {:e} not below tol {:e}",
        step.final_residual_norm,
        cfg.tol,
    );
}

// =============================================================================
// verify_iter_count_exact
// =============================================================================

/// `iter_count` matches [`N_NEWTON_ITER_EXACT`] exactly. Stronger
/// regression net than `iter_count < cfg.max_newton_iter` —
/// [`verify_newton_convergence`] above already covers the bound, this
/// pin catches a silent change in Newton convergence rate even when
/// the budget still passes. Companion to [`verify_x_final_bit_equal`]
/// below; same dense-path bit-stability contract.
fn verify_iter_count_exact(step: &NewtonStep<CpuTape>) {
    assert_eq!(
        step.iter_count, N_NEWTON_ITER_EXACT,
        "iter_count drift on the canonical θ = {THETA} N step — got {}, expected \
         {N_NEWTON_ITER_EXACT}. Newton convergence rate moved on a path that's bit-stable \
         on a fixed toolchain. Diagnose with the same protocol as the bit-equal anchor: \
         (1) rule out toolchain drift; (2) identify the regressing commit; \
         (3) do NOT re-bake.",
        step.iter_count,
    );
}

// =============================================================================
// verify_x_final_length
// =============================================================================

/// `step.x_final` carries 12 entries (4 vertices × 3 DOFs in
/// vertex-major + xyz-inner layout). Catches a return-shape regression
/// at the trait layer.
fn verify_x_final_length(step: &NewtonStep<CpuTape>) {
    assert_eq!(
        step.x_final.len(),
        N_DOF,
        "x_final length drift: got {}, expected {N_DOF} (4 vertices × 3 DOFs)",
        step.x_final.len(),
    );
}

// =============================================================================
// verify_pinned_unmoved
// =============================================================================

/// Every pinned DOF (indices `0..N_PINNED_DOFS` covering `v_0..v_2`'s
/// xyz) sits within `PINNED_DOF_BOUND_M` of its rest position. Anchors
/// the `BoundaryConditions::pinned_vertices` Dirichlet contract.
fn verify_pinned_unmoved(step: &NewtonStep<CpuTape>, initial: &SceneInitial) {
    let x_prev = initial.x_prev.as_slice();
    for (i, (&got, &expected)) in step
        .x_final
        .iter()
        .zip(x_prev.iter())
        .take(N_PINNED_DOFS)
        .enumerate()
    {
        assert!(
            (got - expected).abs() < PINNED_DOF_BOUND_M,
            "Dirichlet DOF {i} drifted: x_final[{i}] = {got:e} vs x_prev[{i}] = {expected:e}, \
             |Δ| = {:e} ≥ {PINNED_DOF_BOUND_M:e}",
            (got - expected).abs(),
        );
    }
}

// =============================================================================
// verify_loaded_z_positive
// =============================================================================

/// `v_3.z` displacement is strictly positive — force is `+ẑ`, so the
/// loaded vertex must move in the force direction. Catches a
/// sign-flip regression in the load-axis dispatch
/// (`backward_euler.rs:807-817`).
fn verify_loaded_z_positive(step: &NewtonStep<CpuTape>, initial: &SceneInitial) -> f64 {
    let dz = step.x_final[V3_Z_DOF] - initial.x_prev.as_slice()[V3_Z_DOF];
    assert!(
        dz > 0.0,
        "v_3.z must move in +ẑ under +ẑ traction, got dz = {dz:e}",
    );
    dz
}

// =============================================================================
// verify_loaded_axis_alignment
// =============================================================================

/// `v_3.x` and `v_3.y` displacements are below `DXY_BOUND_M`. F=I
/// isotropic-tangent claim — at the rest configuration the NH tangent
/// is diagonal in the canonical axis-aligned tet, so the Newton step
/// is pure `+ẑ` at the linear level. NH geometric nonlinearity at
/// the ~1% strain regime here induces at most `O(strain² · dz)`
/// ≈ `1e-7` m off-axis displacement, two orders below `DXY_BOUND_M`.
fn verify_loaded_axis_alignment(step: &NewtonStep<CpuTape>, initial: &SceneInitial) -> (f64, f64) {
    let dx = step.x_final[V3_X_DOF] - initial.x_prev.as_slice()[V3_X_DOF];
    let dy = step.x_final[V3_Y_DOF] - initial.x_prev.as_slice()[V3_Y_DOF];
    assert!(
        dx.abs() < DXY_BOUND_M,
        "v_3.x = {dx:e} — expected near-zero at F=I isotropic tangent (< {DXY_BOUND_M:e} m)",
    );
    assert!(
        dy.abs() < DXY_BOUND_M,
        "v_3.y = {dy:e} — expected near-zero at F=I isotropic tangent (< {DXY_BOUND_M:e} m)",
    );
    (dx, dy)
}

// =============================================================================
// verify_dimensional_band
// =============================================================================

/// `v_3.z` displacement sits inside the dimensional-analysis band
/// `[DZ_LOWER_BOUND_M, DZ_UPPER_BOUND_M]`. Centered on the linear
/// estimate `δz ≈ 9.6e-4` m with ~30% NH-nonlinearity margin on each
/// side. Catches a constitutive- or assembly-scale regression that
/// would shift the response by multiple band-widths.
fn verify_dimensional_band(dz: f64) {
    assert!(
        (DZ_LOWER_BOUND_M..=DZ_UPPER_BOUND_M).contains(&dz),
        "dz = {dz:e} outside dimensional-analysis band [{DZ_LOWER_BOUND_M:e}, \
         {DZ_UPPER_BOUND_M:e}] m at θ = {THETA} N (linear estimate \
         {ANALYTIC_LINEAR_ESTIMATE_M:e} m)",
    );
}

// =============================================================================
// verify_x_final_bit_equal
// =============================================================================

/// Every entry of `step.x_final` matches the IV-1 captured bit
/// pattern at sim-soft `c3729d4a`. See the comment block on
/// [`ONE_TET_X_FINAL_BITS`] for the capture provenance + two-tier
/// contract + failure-mode protocol — the strongest regression net
/// available on this dense small-FEM path.
fn verify_x_final_bit_equal(step: &NewtonStep<CpuTape>) {
    for (i, (&val, &expected_bits)) in step
        .x_final
        .iter()
        .zip(ONE_TET_X_FINAL_BITS.iter())
        .enumerate()
    {
        let got_bits = val.to_bits();
        assert_eq!(
            got_bits,
            expected_bits,
            "x_final[{i}] bit drift vs IV-1 reference — got {got_bits:#018x} ({val:e}), \
             expected {expected_bits:#018x} ({:e}). See [`ONE_TET_X_FINAL_BITS`] doc \
             comment for the failure-mode protocol — do NOT re-bake without ruling out \
             a real toolchain delta and a real regression first.",
            f64::from_bits(expected_bits),
        );
    }
}

// =============================================================================
// JSON emit — single-record force-stretch trace
// =============================================================================

/// Single-step record at `θ = THETA` capturing the full DOF state +
/// reduced `v_3` displacement readout. The constitutive `θ`-sweep
/// curve belongs to row 5 (`neo-hookean-uniaxial`); this row's JSON
/// is the inspection artifact for the walking-skeleton step API
/// itself, not a force-stretch curve.
fn save_force_stretch_json(
    step: &NewtonStep<CpuTape>,
    initial: &SceneInitial,
    cfg: &SolverConfig,
    displacement_v3: (f64, f64, f64),
    path: &Path,
) -> Result<()> {
    let (dx, dy, dz) = displacement_v3;
    let x_prev: Vec<f64> = initial.x_prev.as_slice().to_vec();
    let record = json!({
        "scene": {
            "mesh_corners_m": [
                [0.0, 0.0, 0.0],
                [0.1, 0.0, 0.0],
                [0.0, 0.1, 0.0],
                [0.0, 0.0, 0.1],
            ],
            "boundary_conditions": {
                "pinned_vertices": [0, 1, 2],
                "loaded_vertices": [{"vertex": 3, "axis": "AxisZ"}],
            },
            "material_uniform": {
                "mu_Pa": MU_PA,
                "lambda_Pa": LAMBDA_PA,
            },
            "density_kg_m3": DENSITY_KG_M3,
            "dt_s": cfg.dt,
            "tol": cfg.tol,
            "max_newton_iter": cfg.max_newton_iter,
        },
        "input": {
            "theta_N": THETA,
        },
        "step": {
            "iter_count": step.iter_count,
            "final_residual_norm": step.final_residual_norm,
            "x_prev": x_prev,
            "x_final": step.x_final,
        },
        "displacement_v3": {
            "dx_m": dx,
            "dy_m": dy,
            "dz_m": dz,
            "analytic_linear_estimate_m": ANALYTIC_LINEAR_ESTIMATE_M,
            "dimensional_band_m": [DZ_LOWER_BOUND_M, DZ_UPPER_BOUND_M],
        },
    });

    let file = std::fs::File::create(path)
        .with_context(|| format!("failed to create {}", path.display()))?;
    serde_json::to_writer_pretty(&file, &record)
        .with_context(|| format!("failed to serialize record to {}", path.display()))?;
    Ok(())
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

fn print_summary(
    step: &NewtonStep<CpuTape>,
    cfg: &SolverConfig,
    displacement_v3: (f64, f64, f64),
    path: &Path,
) {
    let (dx, dy, dz) = displacement_v3;
    println!("==== single-tet-stretch ====");
    println!();
    println!("input  : SoftScene::one_tet_cube()");
    println!("         vertices: v_0=(0,0,0), v_1=(0.1,0,0), v_2=(0,0.1,0), v_3=(0,0,0.1)  m");
    println!("         material: μ = {MU_PA:e} Pa, λ = {LAMBDA_PA:e} Pa  (uniform)");
    println!("         density : ρ = {DENSITY_KG_M3} kg/m³  (silicone-class)");
    println!("         BC      : pin v_0, v_1, v_2 (full Dirichlet)");
    println!("                   load v_3 along LoadAxis::AxisZ");
    println!(
        "         step    : Δt = {} s, tol = {:e}, max_newton_iter = {}",
        cfg.dt, cfg.tol, cfg.max_newton_iter
    );
    println!("         θ       : {THETA} N along +ẑ on v_3");
    println!();
    println!("Anchor groups (all assertions exit-0 on success):");
    println!("  newton_convergence       : iter_count < max_newton_iter; residual < tol");
    println!(
        "  iter_count_exact         : iter_count == {N_NEWTON_ITER_EXACT} (bit-stable on dense path)"
    );
    println!("  x_final_length           : 12 DOFs (4 vertices × 3)");
    println!(
        "  pinned_unmoved           : every pinned DOF within {PINNED_DOF_BOUND_M:e} m of rest"
    );
    println!("  loaded_z_positive        : v_3.z displacement strictly positive");
    println!("  loaded_axis_alignment    : v_3.x, v_3.y displacements < {DXY_BOUND_M:e} m");
    println!("  dimensional_band         : v_3.z ∈ [{DZ_LOWER_BOUND_M:e}, {DZ_UPPER_BOUND_M:e}] m");
    println!("  x_final_bit_equal        : every DOF matches IV-1 reference bits (c3729d4a)");
    println!();
    println!("Step result:");
    println!("  iter_count               : {:>3}", step.iter_count);
    println!(
        "  final_residual_norm      : {:>13e}  (tol {:e})",
        step.final_residual_norm, cfg.tol
    );
    println!();
    println!("v_3 displacement (loaded vertex):");
    println!("  dx                       : {dx:>13e} m");
    println!("  dy                       : {dy:>13e} m");
    println!("  dz                       : {dz:>13e} m");
    println!("  linear estimate (δz)     : {ANALYTIC_LINEAR_ESTIMATE_M:>13e} m  (θ / A_33,zz)");
    println!("  dimensional band         : [{DZ_LOWER_BOUND_M:e}, {DZ_UPPER_BOUND_M:e}] m");
    println!();
    println!("JSON   : {}", path.display());
    println!("         single-record force-stretch trace — full DOF state + v_3 readout");
    println!("         (constitutive θ-sweep is row 5 `neo-hookean-uniaxial`)");
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    let (step, initial, cfg) = run_skeleton_step();

    verify_newton_convergence(&step, &cfg);
    verify_iter_count_exact(&step);
    verify_x_final_length(&step);
    verify_pinned_unmoved(&step, &initial);
    let dz = verify_loaded_z_positive(&step, &initial);
    let (dx, dy) = verify_loaded_axis_alignment(&step, &initial);
    verify_dimensional_band(dz);
    verify_x_final_bit_equal(&step);

    let displacement_v3 = (dx, dy, dz);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)
        .with_context(|| format!("failed to create {}", out_dir.display()))?;
    let out_path = out_dir.join("force_stretch.json");
    save_force_stretch_json(&step, &initial, &cfg, displacement_v3, &out_path)?;

    print_summary(&step, &cfg, displacement_v3, &out_path);

    Ok(())
}
