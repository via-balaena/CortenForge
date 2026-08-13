//! ▶ **PILOT — can a flexing Tet10 stick fit a VR frame?**
//!
//! Two questions that are usually asked separately and must not be:
//!
//! 1. **Speed.** How long does one implicit step take, against the **11.1 ms**
//!    a 90 Hz headset allows for *everything* — physics, render, both eyes.
//! 2. **Fidelity.** How many elements does the flex actually *need* before the
//!    answer stops moving?
//!
//! ⚠⚠ Asking (1) alone is how a mesh gets called "fast enough" while being too
//! coarse to bend correctly. A hockey stick's whip and kick point are exactly
//! what a player feels, so a stick that fits the budget and flexes like a
//! fishing rod has answered the wrong question. **The number that matters is
//! the cost at the element count where deflection has converged**, not the cost
//! at whatever count someone tried first.
//!
//! # What is measured
//!
//! A cantilever driven by a tip load — the same fixture `tet10_bending_locking`
//! uses, which is this crate's canonical bending case and a reasonable stick
//! proxy: clamped at one end, loaded at the other, bending dominated.
//!
//! ⚠ It is a **solid isotropic** shaft at hockey-stick proportions (1.5 m ×
//! 30 mm), not a hollow tapered composite lay-up. The cost curve transfers —
//! it is set by DOF count and conditioning — but the *converged element count*
//! for a real stick may differ, because a thin-walled section needs elements
//! through the wall that a solid one does not. Read the fidelity column as
//! "how fast does tip deflection settle for a slender bending beam", which is
//! the right question one level of abstraction up from the actual part.
//!
//! Per refinement level: elements, DOF, **ms per implicit step**, Newton
//! iterations, ms per iteration, and **tip deflection**.
//!
//! The iteration split matters for planning. A step that is slow because it
//! takes *many* iterations is tunable (warm starts, looser tolerance, smaller
//! `dt`); a step whose *individual* iterations are slow is a factorization cost
//! and needs a different formulation — reduced-order, GPU, or explicit.
//!
//! # ⚠ This is a PESSIMISTIC bound, deliberately
//!
//! Every step here is quasi-static from rest with a stiff tolerance, which is
//! what the disc work needed. A real-time loop steps dynamically with a small
//! `dt` and a **previous-frame warm start**, which is a materially easier
//! problem — the same solve arrives already near its answer. So the achievable
//! number is better than what this prints, and the gap is where the slack
//! lives. It is stated here rather than quietly claimed.

#![allow(
    // Element/DOF counts are exact well below 2^53; printed ratios are report
    // lines, not thresholds.
    clippy::cast_precision_loss,
    // Mesh construction failing is a regression worth a panic.
    clippy::expect_used
)]

use std::time::Instant;

use sim_ml_chassis::Tensor;
use sim_soft::element::Tet10;
use sim_soft::mesh::HandBuiltTetMesh;
use sim_soft::solver::CpuNewtonSolver;
use sim_soft::{
    BoundaryConditions, CpuTet10NHSolver, LoadAxis, MaterialField, Mesh, NullContact, Solver,
    SolverConfig, Tet10Mesh, VertexId, pick_vertices_by_predicate,
};

/// Frame budget at 90 Hz, in milliseconds — for *everything*, not just physics.
const FRAME_BUDGET_MS: f64 = 1000.0 / 90.0;

// A slender shaft: 1.5 m long, 30 mm across — hockey-stick proportioned, so the
// aspect ratio (and therefore the bending conditioning) is representative.
const LENGTH: f64 = 1.5;
const BREADTH: f64 = 0.03;
const HEIGHT: f64 = 0.03;
/// Shear modulus. Stiff enough to be a composite shaft rather than a noodle;
/// the absolute value cancels out of the convergence question.
const MU: f64 = 2.0e9;
const NU: f64 = 0.35;
const TIP_FORCE_TOTAL: f64 = 150.0;
const STATIC_DT: f64 = 1.0;
/// Increments the tip load is ramped over. Each is one timed "frame".
const RAMP_STEPS: usize = 10;

/// `(elements, dof, ms_per_step, newton_iters, tip_deflection_m)` for one
/// refinement of the shaft.
fn measure(nx: usize, ny: usize, nz: usize, tol_rel: f64) -> (usize, usize, f64, usize, f64) {
    let lambda = 2.0 * MU * NU / (1.0 - 2.0 * NU);
    let field = MaterialField::uniform(MU, lambda);
    let tet4 =
        HandBuiltTetMesh::cantilever_bilayer_beam(nx, ny, nz, LENGTH, BREADTH, HEIGHT, &field);
    let mesh = Tet10Mesh::from_tet4(&tet4);
    let n_elems = mesh.n_tets();
    let n_dof = 3 * mesh.n_vertices();

    let pinned: Vec<VertexId> = pick_vertices_by_predicate(&mesh, |p| p.x.abs() < 1e-9);
    let loaded: Vec<VertexId> = pick_vertices_by_predicate(&mesh, |p| (p.x - LENGTH).abs() < 1e-9);
    assert!(
        !pinned.is_empty() && !loaded.is_empty(),
        "clamped and tip bands must be non-empty"
    );

    let positions = mesh.positions();
    let mut x_flat = vec![0.0; n_dof];
    for (v, p) in positions.iter().enumerate() {
        x_flat[3 * v] = p.x;
        x_flat[3 * v + 1] = p.y;
        x_flat[3 * v + 2] = p.z;
    }
    let rest_z: Vec<f64> = loaded.iter().map(|&v| positions[v as usize].z).collect();

    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: Vec::new(),
        loaded_vertices: loaded.iter().map(|&v| (v, LoadAxis::AxisZ)).collect(),
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = 500;
    // ⚠ Scaled to the load, as `tet10_bending_locking` does. `skeleton()`'s
    // default is sized for the walking-skeleton scene; leaving it on a 2 GPa
    // shaft asks for a residual the solve cannot reach.
    cfg.tol = tol_rel * TIP_FORCE_TOTAL / (loaded.len() as f64).sqrt();
    let solver: CpuTet10NHSolver<Tet10Mesh> =
        CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc);

    let v_prev = Tensor::zeros(&[n_dof]);

    // ⚠⚠ The load is RAMPED, not applied in one shot.
    //
    // A first cut asked for the full tip load from rest in a single quasi-static
    // solve — a ~46 cm deflection in one step — and every refinement diverged.
    // That is the same mistake as the rung-5 defect: demanding the entire
    // transit at once. It is also the wrong thing to measure: a real-time loop
    // never does that. It advances a small increment per frame from the previous
    // converged state, which is exactly what this now times.
    //
    // ⇒ `ms/step` below is the cost of ONE INCREMENT — the per-frame cost.
    let mut x_prev = Tensor::from_slice(&x_flat, &[n_dof]);
    let mut best = f64::INFINITY;
    let mut iters = 0usize;
    let mut tip = 0.0f64;
    for i in 1..=RAMP_STEPS {
        let load = TIP_FORCE_TOTAL * (i as f64 / RAMP_STEPS as f64);
        let theta = Tensor::from_slice(&[load / loaded.len() as f64], &[1]);
        let clock = Instant::now();
        // `try_` so a grid that diverges REPORTS rather than killing the whole
        // curve — a refinement that cannot take this load is a row of the
        // answer, not a reason to lose the other rows.
        let step = match solver.try_replay_step(&x_prev, &v_prev, &theta, cfg.dt) {
            Ok(st) => st,
            Err(e) => {
                let msg = format!("{e:?}");
                println!(
                    "    PROBE {nx}x{ny}x{nz} increment {i} failed: {}",
                    &msg[..msg.len().min(260)]
                );
                return (n_elems, n_dof, f64::NAN, 0, f64::NAN);
            }
        };
        let ms = clock.elapsed().as_secs_f64() * 1e3;
        // The first increment pays first-touch page faults and any lazy init;
        // steady-state per-frame cost is what a budget is about.
        if i > 1 {
            best = best.min(ms);
        }
        iters = step.iter_count;
        tip = loaded
            .iter()
            .zip(&rest_z)
            .map(|(&v, &z0)| (step.x_final[3 * v as usize + 2] - z0).abs())
            .fold(0.0f64, f64::max);
        x_prev = Tensor::from_slice(&step.x_final, &[n_dof]);
    }
    (n_elems, n_dof, best, iters, tip)
}

#[test]
#[cfg_attr(debug_assertions, ignore = "release-only timing measurement")]
fn realtime_budget_curve_for_a_flexing_tet10_shaft() {
    println!(
        "\n=== Tet10 shaft: cost vs fidelity (90 Hz budget = {FRAME_BUDGET_MS:.2} ms/frame) ==="
    );
    println!(
        "  ⚠ TOLERANCE is the axis, not a detail. `tet10_bending_locking` expects up to 350\n  \
         Newton iterations at 1e-8 relative — this solver was built for the disc's accuracy,\n  \
         not for latency. A game needs VISUALLY converged, not 1e-8, and the question is what\n  \
         that buys."
    );

    // ⚠ ONE variable at a time within each block: refinement along the LENGTH
    // only, cross-section fixed at 2x2. A sequence that also thickened the
    // section would make "vs prev" compare meshes differing in two ways — the
    // error the rung5 investigation turned on.
    let grids = [(8, 2, 2), (16, 2, 2), (24, 2, 2), (32, 2, 2)];

    for tol_rel in [1e-8, 1e-5, 1e-3] {
        println!("\n  relative tolerance {tol_rel:e}");
        println!(
            "{:>12} {:>8} {:>8} {:>10} {:>7} {:>9} {:>12} {:>9}",
            "grid", "elems", "dof", "ms/step", "iters", "ms/iter", "tip (mm)", "vs prev"
        );
        let mut prev_tip: Option<f64> = None;
        for (nx, ny, nz) in grids {
            let (elems, dof, ms, iters, tip) = measure(nx, ny, nz, tol_rel);
            let delta = prev_tip.map_or(f64::INFINITY, |p: f64| {
                (tip - p).abs() / p.abs().max(1e-12) * 100.0
            });
            let fits = if ms.is_finite() && ms < FRAME_BUDGET_MS {
                " ✓fits"
            } else {
                ""
            };
            println!(
                "{:>12} {elems:>8} {dof:>8} {ms:>10.2} {iters:>7} {:>9.2} {:>12.3} {}{fits}",
                format!("{nx}x{ny}x{nz}"),
                ms / (iters.max(1) as f64),
                tip * 1e3,
                if delta.is_finite() {
                    format!("{delta:>8.2}%")
                } else {
                    "        —".to_string()
                },
            );
            prev_tip = Some(tip);
        }
    }

    println!(
        "\n  'vs prev' is the FIDELITY column — where it stops moving is the mesh the physics\n  \
         needs, and the ms/step beside it is the cost that actually has to fit. Compare the\n  \
         tip across TOLERANCE blocks too: if a loose tolerance gives the same deflection, the\n  \
         iterations it saved were buying precision nobody can see.\n  \
         ⚠ Load is ramped over {RAMP_STEPS} increments and each is timed from the previous\n  \
         converged state — the per-FRAME cost, which is what a real-time loop pays. NaN = the\n  \
         Newton iteration cap, i.e. that tolerance is unreachable for that mesh."
    );
}
