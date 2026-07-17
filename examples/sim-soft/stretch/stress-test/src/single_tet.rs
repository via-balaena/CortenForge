//! single-tet — `SkeletonSolver` end-to-end on `SoftScene::one_tet_cube`.
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
//! This module is a DEMONSTRATION: it drives the real solver one step
//! and emits the resulting DOF trace as JSON — the inspection artifact
//! for the walking-skeleton step API (no spatial artifact, no `cf-view`:
//! single-tet has trivial topology, per inventory Q4 row 4).
//!
//! **Correctness lives in the library.** The physical contract of this
//! exact scene — `x_final` values (bit-equal), Dirichlet-pin enforcement,
//! `+ẑ` displacement sign, F=I axis alignment, and the dimensional-
//! analysis displacement band — is owned by `sim-soft`'s own lib tests:
//! `solver_convergence.rs::stage_1_traction_converges` (the byte-for-byte
//! canonical scene) and `invariant_iv_1_uniform_passthrough.rs`'s
//! `ONE_TET_X_FINAL` bit-equal contract. This module therefore self-gates
//! only on demonstration-integrity: that the solve CONVERGED, so the
//! emitted trace is a real converged solution — not a silent garbage
//! state. (It does NOT re-pin `iter_count`: the lib deliberately uses a
//! range, `iter_count < max_newton_iter`, rather than an exact freeze.)

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

/// `v_3`'s z-DOF index in the vertex-major + xyz-inner layout
/// (`v_3` = vertex 3, z is component 2 → index `3·3 + 2 = 11`).
const V3_Z_DOF: usize = 11;

/// `v_3`'s x-DOF index (`3·3 + 0 = 9`).
const V3_X_DOF: usize = 9;

/// `v_3`'s y-DOF index (`3·3 + 1 = 10`).
const V3_Y_DOF: usize = 10;

// =============================================================================
// Builder
// =============================================================================

/// Build the canonical scene + skeleton solver, run one `Solver::step`
/// at `θ = THETA` along `+ẑ` on `v_3`, return the step result + the
/// initial scene state + the solver config (the latter two threaded
/// through to the convergence gate and the JSON / stdout summaries).
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
// verify_newton_convergence — demonstration-integrity gate
// =============================================================================

/// Newton converged within budget AND the final residual sits below the
/// configured tolerance — so the emitted `x_final` trace is a real
/// converged solution, not a silent non-converged state. Anchors
/// `Solver::step`'s convergence contract (scope §3 R-1: 3-5 iter from
/// rest at Stage-1 θ; loop exits via convergence return, so
/// `iter_count < max_newton_iter` always — equality is unreachable on a
/// converged path). The detailed physical correctness of the converged
/// state is owned by `solver_convergence.rs` + IV-1 (see module docstring).
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

/// `v_3` displacement `(dx, dy, dz)` — the loaded vertex's motion vs its
/// rest position, read straight from the converged `x_final`.
fn loaded_displacement(step: &NewtonStep<CpuTape>, initial: &SceneInitial) -> (f64, f64, f64) {
    let x_prev = initial.x_prev.as_slice();
    let dx = step.x_final[V3_X_DOF] - x_prev[V3_X_DOF];
    let dy = step.x_final[V3_Y_DOF] - x_prev[V3_Y_DOF];
    let dz = step.x_final[V3_Z_DOF] - x_prev[V3_Z_DOF];
    (dx, dy, dz)
}

// =============================================================================
// JSON emit — single-record force-stretch trace
// =============================================================================

/// Single-step record at `θ = THETA` capturing the full DOF state +
/// reduced `v_3` displacement readout. The constitutive `θ`-sweep
/// curve belongs to row 5 (the `neo_hookean` module); this row's JSON
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
    println!("==== single_tet ====");
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
    println!("Demonstration gate (physical correctness owned by solver_convergence.rs + IV-1):");
    println!("  newton_convergence : iter_count < max_newton_iter; residual < tol");
    println!();
    println!("Step result (real solver output):");
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
    println!();
    println!("JSON   : {}", path.display());
    println!("         single-record step trace — full DOF state + v_3 readout");
    println!("         (constitutive θ-sweep is row 5 `neo_hookean`)");
}

// =============================================================================
// main
// =============================================================================

pub fn run() -> Result<()> {
    let (step, initial, cfg) = run_skeleton_step();

    verify_newton_convergence(&step, &cfg);
    let displacement_v3 = loaded_displacement(&step, &initial);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("out")
        .join("single_tet");
    std::fs::create_dir_all(&out_dir)
        .with_context(|| format!("failed to create {}", out_dir.display()))?;
    let out_path = out_dir.join("force_stretch.json");
    save_force_stretch_json(&step, &initial, &cfg, displacement_v3, &out_path)?;

    print_summary(&step, &cfg, displacement_v3, &out_path);

    Ok(())
}
