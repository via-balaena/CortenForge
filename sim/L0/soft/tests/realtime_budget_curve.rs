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

// ---------------------------------------------------------------------------
// Which variable kills it?
// ---------------------------------------------------------------------------

/// ▶ **Separate ASPECT RATIO from STIFFNESS.**
///
/// `realtime_budget_curve_for_a_flexing_tet10_shaft` cannot drive a
/// stick-proportioned shaft at any tolerance — Newton hits the iteration cap at
/// ~0.1 % of the expected deflection, and the partial states are bit-identical
/// across `1e-8`/`1e-5`/`1e-3`, so tolerance is not the binding constraint.
///
/// The known-good control already ships: `tet10_bending_locking` converges on a
/// **5:1** aspect beam at **μ = 1e5**. The failing shaft is **50:1** at
/// **μ = 2e9**. ⚠ **Two variables**, and guessing which one matters is the exact
/// error the rung-5 investigation turned on — so they are walked independently.
///
/// # Holding the physics comparable
///
/// Load is not held fixed; the *relative deflection* is. For a cantilever,
/// `δ = F L³ / (3 E I)` with `I = h⁴/12` and `E = 2μ(1+ν)`, so targeting
/// `δ/L = r` needs
///
/// ```text
///     F = μ (1 + ν) h⁴ r / (2 L²)
/// ```
///
/// Without that, a row could fail merely because it was pushed harder, and the
/// table would be comparing rows that differ in two ways again.
fn separation_row(aspect: f64, mu: f64) -> (f64, usize, f64, bool) {
    separation_row_at(aspect, mu, 2, false)
}

/// [`separation_row`] with the through-thickness refinement and F-bar exposed —
/// the two candidate remedies for the slender-beam stiffness.
fn separation_row_at(aspect: f64, mu: f64, nz: usize, fbar: bool) -> (f64, usize, f64, bool) {
    const L: f64 = 1.5;
    const TARGET_RATIO: f64 = 0.10;
    let h = L / aspect;
    let nu = 0.35;
    let load = mu * (1.0 + nu) * h.powi(4) * TARGET_RATIO / (2.0 * L * L);

    let lambda = 2.0 * mu * nu / (1.0 - 2.0 * nu);
    let field = MaterialField::uniform(mu, lambda);
    let tet4 = HandBuiltTetMesh::cantilever_bilayer_beam(8, 2, nz, L, h, h, &field);
    let mesh = Tet10Mesh::from_tet4(&tet4);
    let n_dof = 3 * mesh.n_vertices();

    let pinned: Vec<VertexId> = pick_vertices_by_predicate(&mesh, |p| p.x.abs() < 1e-9);
    let loaded: Vec<VertexId> = pick_vertices_by_predicate(&mesh, |p| (p.x - L).abs() < 1e-9);
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
    cfg.tol = 1e-6 * load / (loaded.len() as f64).sqrt();
    cfg.fbar = fbar;
    let solver: CpuTet10NHSolver<Tet10Mesh> =
        CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc);

    let x_prev = Tensor::from_slice(&x_flat, &[n_dof]);
    let v_prev = Tensor::zeros(&[n_dof]);
    let theta = Tensor::from_slice(&[load / loaded.len() as f64], &[1]);

    let clock = Instant::now();
    match solver.try_replay_step(&x_prev, &v_prev, &theta, cfg.dt) {
        Ok(step) => {
            let ms = clock.elapsed().as_secs_f64() * 1e3;
            let tip = loaded
                .iter()
                .zip(&rest_z)
                .map(|(&v, &z0)| (step.x_final[3 * v as usize + 2] - z0).abs())
                .fold(0.0f64, f64::max);
            (ms, step.iter_count, tip / L, true)
        }
        Err(_) => (f64::NAN, 0, f64::NAN, false),
    }
}

#[test]
#[cfg_attr(debug_assertions, ignore = "release-only timing measurement")]
fn which_variable_stops_the_shaft_converging() {
    println!("\n=== Separation: aspect ratio vs stiffness, one at a time ===");
    println!("  every row is loaded to the SAME target deflection (δ/L = 0.10),");
    println!("  so no row fails merely because it was pushed harder.\n");

    println!("  (A) ASPECT swept, stiffness held at the known-good μ = 1e5");
    println!(
        "{:>10} {:>10} {:>8} {:>10} {:>10}",
        "aspect", "ms", "iters", "δ/L", "converged"
    );
    let mut aspect_ok = Vec::new();
    for aspect in [5.0, 10.0, 20.0, 40.0] {
        let (ms, iters, ratio, ok) = separation_row(aspect, 1.0e5);
        println!("{aspect:>10.0} {ms:>10.2} {iters:>8} {ratio:>10.4} {ok:>10}");
        aspect_ok.push((aspect, ok));
    }

    println!("\n  (B) STIFFNESS swept, aspect held at the known-good 5:1");
    println!(
        "{:>10} {:>10} {:>8} {:>10} {:>10}",
        "mu", "ms", "iters", "δ/L", "converged"
    );
    let mut mu_ok = Vec::new();
    for mu in [1.0e5, 1.0e7, 1.0e9, 2.0e9] {
        let (ms, iters, ratio, ok) = separation_row(5.0, mu);
        println!("{mu:>10.0e} {ms:>10.2} {iters:>8} {ratio:>10.4} {ok:>10}");
        mu_ok.push((mu, ok));
    }

    println!("\n  (C) the corner — stick-proportioned AND stick-stiff");
    let (ms, iters, ratio, ok) = separation_row(40.0, 2.0e9);
    println!(
        "{:>10} {ms:>10.2} {iters:>8} {ratio:>10.4} {ok:>10}",
        "40 @ 2e9"
    );

    println!(
        "\n  ⚠ δ/L should land near 0.10 wherever it converged. A converged row that\n  \
         MISSES the target is worse news than a failed one: it means the solve returned\n  \
         an answer that is not the beam's."
    );

    // The control must hold, or the whole table is about a broken harness rather
    // than about aspect ratio and stiffness.
    assert!(
        aspect_ok[0].1 && mu_ok[0].1,
        "the known-good config (5:1 at mu = 1e5) must converge — it is what \
         `tet10_bending_locking` ships. If it does not, this harness is wrong, not the solver"
    );
}

/// ▶ **Two candidate remedies for the slender-beam stiffness, measured.**
///
/// The separation showed a stick-proportioned beam converging to an answer that
/// is not the beam's: `δ/L = 0.0090` against a target of `0.10` at 40:1, while
/// reporting success. Stiffness was free; aspect ratio was the whole effect.
///
/// ⚠⚠ **F-bar is not available at all.** It was the obvious reach — but it
/// panics for Tet10 by design: *"F-bar's single-Gauss-point volumetric
/// constraint has no multi-Gauss-point Tet10 analog"* (`newton.rs`). It is a
/// **Tet4-only** remedy, so the element a bending stick actually needs cannot
/// use it. That also fits the diagnosis: `fbar_locking` cures **volumetric**
/// locking at `ν = 0.49`, and this beam runs at `ν = 0.35` with 10:1 *element*
/// aspect ratios — **shear** locking, a different pathology.
///
/// ⇒ **Through-thickness refinement is the only candidate remaining in the
/// tree**, and it attacks element aspect ratio directly.
///
/// Whichever recovers `δ/L` toward 0.10 is the answer, and if the winner is
/// through-thickness refinement then the cost lands straight back on the frame
/// budget — which is the honest shape of the trade.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only timing measurement")]
fn does_fbar_or_thickness_refinement_recover_the_slender_beam() {
    println!("\n=== Recovering a slender beam: F-bar vs through-thickness refinement ===");
    println!("  target δ/L = 0.10 everywhere; 40:1 is stick-proportioned.\n");
    println!(
        "{:>8} {:>6} {:>7} {:>10} {:>8} {:>10} {:>10}",
        "aspect", "nz", "fbar", "ms", "iters", "δ/L", "converged"
    );

    for aspect in [5.0, 20.0, 40.0] {
        // ⚠ F-bar is NOT in this matrix, and not because it was untried: it panics
        // for Tet10 by design — "F-bar's single-Gauss-point volumetric constraint
        // has no multi-Gauss-point Tet10 analog" (`newton.rs`). It is a Tet4-only
        // remedy, so the element a bending stick needs cannot use it at all.
        // Through-thickness refinement is the only candidate left in the tree.
        for (nz, fbar) in [(2, false), (4, false), (6, false), (8, false)] {
            let (ms, iters, ratio, ok) = separation_row_at(aspect, 1.0e5, nz, fbar);
            println!(
                "{aspect:>8.0} {nz:>6} {:>7} {ms:>10.2} {iters:>8} {ratio:>10.4} {ok:>10}",
                if fbar { "on" } else { "off" }
            );
        }
        println!();
    }

    println!(
        "  Read the 40:1 block. If δ/L climbs toward 0.10 with nz, the pathology is element\n  \
         aspect ratio and the fix costs DOF — which puts the frame budget back at the centre.\n  \
         If it does NOT climb, a solid Tet10 is the wrong element for a stick at this\n  \
         slenderness, and the answer is a beam/shell formulation rather than more mesh."
    );

    // The control: the known-good 5:1 must still land near target, or this table
    // is measuring a broken harness.
    let (_, _, base_ratio, base_ok) = separation_row_at(5.0, 1.0e5, 2, false);
    assert!(
        base_ok && base_ratio > 0.05,
        "the known-good 5:1 beam must converge near target (got δ/L = {base_ratio:.4}) — \
         otherwise this table is about the harness, not the element"
    );
}

// ---------------------------------------------------------------------------
// The validation case
// ---------------------------------------------------------------------------

/// ★★ **Is the element right at slender proportions?** — against an analytic
/// answer, in the regime where that answer is not in doubt.
///
/// The remedy sweep left an unattributed gap: a stick-proportioned beam is
/// mesh-converged (`nz` 2→8 moves `δ/L` by 0.2 %) yet deflects ~11× less than
/// small-deflection beam theory. Two candidate explanations, and **nothing in
/// this repo could separate them** — `tet10_bending_locking` validates a *ratio*
/// (Tet10 vs Tet4, ν-sensitivity) at 5:1, never an **absolute** deflection at
/// slenderness. For a hockey stick, absolute flex *is* the product.
///
/// # Why the LINEAR regime, and not the elastica
///
/// The obvious reach is a published large-deflection (elastica) solution. This
/// deliberately does not: those values would come from memory, and a validation
/// case whose ground truth is half-remembered validates nothing. Instead it
/// drives a deflection small enough that **Euler–Bernoulli is exact to within
/// the measurement**, where the closed form is unambiguous:
///
/// ```text
///     δ = P L³ / (3 E I),   I = h⁴/12,   E = 2μ(1 + ν)
/// ```
///
/// ⚠ The two corrections that could invalidate that, both bounded rather than
/// assumed away:
///
/// - **Shear** (Timoshenko) adds `(1+ν)h² / (2k L²)` relative — **0.05 %** at
///   40:1 and ~3 % at 5:1. Negligible exactly where the question lives.
/// - **Large deflection** stiffens the response, but only above `δ/L ≈ 0.1`;
///   at the `δ/L ≈ 1e-3` driven here it is far below the tolerance.
///
/// ⇒ At 40:1 the analytic answer is trustworthy to well under a percent. **A
/// ratio of 1.0 means the element is right and the earlier 11 % reading was a
/// large-deflection/target artefact; a ratio far from 1.0 means the element is
/// wrong at slenderness and a solid Tet10 is not the way to simulate a stick.**
fn linear_regime_ratio(aspect: f64, nz: usize) -> (f64, f64, bool) {
    const L: f64 = 1.5;
    /// Small enough that Euler–Bernoulli is exact to well under a percent.
    const TARGET_RATIO: f64 = 1.0e-3;
    let h = L / aspect;
    let (mu, nu) = (1.0e5, 0.35);
    // Invert δ = P L³ / (3 E I) for the load that lands on TARGET_RATIO.
    let e = 2.0 * mu * (1.0 + nu);
    let i_area = h.powi(4) / 12.0;
    let load = TARGET_RATIO * L * 3.0 * e * i_area / L.powi(3);
    let delta_analytic = TARGET_RATIO * L;

    let lambda = 2.0 * mu * nu / (1.0 - 2.0 * nu);
    let field = MaterialField::uniform(mu, lambda);
    let tet4 = HandBuiltTetMesh::cantilever_bilayer_beam(8, 2, nz, L, h, h, &field);
    let mesh = Tet10Mesh::from_tet4(&tet4);
    let n_dof = 3 * mesh.n_vertices();

    let pinned: Vec<VertexId> = pick_vertices_by_predicate(&mesh, |p| p.x.abs() < 1e-9);
    let loaded: Vec<VertexId> = pick_vertices_by_predicate(&mesh, |p| (p.x - L).abs() < 1e-9);
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
    cfg.tol = 1e-9 * load / (loaded.len() as f64).sqrt();
    let solver: CpuTet10NHSolver<Tet10Mesh> =
        CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc);

    let theta = Tensor::from_slice(&[load / loaded.len() as f64], &[1]);
    solver
        .try_replay_step(
            &Tensor::from_slice(&x_flat, &[n_dof]),
            &Tensor::zeros(&[n_dof]),
            &theta,
            cfg.dt,
        )
        .map_or((f64::NAN, f64::NAN, false), |step| {
            let tip = loaded
                .iter()
                .zip(&rest_z)
                .map(|(&v, &z0)| (step.x_final[3 * v as usize + 2] - z0).abs())
                .fold(0.0f64, f64::max);
            (tip / delta_analytic, tip, true)
        })
}

#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn tet10_matches_analytic_cantilever_deflection_at_slender_proportions() {
    println!("\n=== VALIDATION: Tet10 vs analytic small-deflection cantilever ===");
    println!("  δ_analytic = P L³ / (3 E I), driven at δ/L ≈ 1e-3 so Euler–Bernoulli is exact.");
    println!("  Shear correction: 0.05 % at 40:1, ~3 % at 5:1 — bounded, not assumed away.\n");
    println!(
        "{:>8} {:>6} {:>14} {:>14} {:>10}",
        "aspect", "nz", "δ_fem (mm)", "δ_exact (mm)", "fem/exact"
    );

    let mut worst_slender = 1.0f64;
    for aspect in [5.0, 10.0, 20.0, 40.0] {
        for nz in [2, 4] {
            let (ratio, tip, ok) = linear_regime_ratio(aspect, nz);
            let exact = 1.0e-3 * 1.5;
            println!(
                "{aspect:>8.0} {nz:>6} {:>14.5} {:>14.5} {:>10}",
                tip * 1e3,
                exact * 1e3,
                if ok {
                    format!("{ratio:.4}")
                } else {
                    "DIVERGED".to_string()
                },
            );
            if ok && aspect >= 20.0 {
                worst_slender = worst_slender.min(ratio.min(1.0 / ratio));
            }
        }
        println!();
    }

    println!(
        "  A ratio near 1.0 at 20:1 and 40:1 means the element is RIGHT at slenderness, and\n  \
         the earlier 11× reading was an artefact of driving δ/L = 0.10 with a small-deflection\n  \
         target. A ratio far from 1.0 means a solid Tet10 does not bend correctly at stick\n  \
         proportions, and no amount of mesh or frame budget fixes that."
    );

    assert!(
        worst_slender > 0.85,
        "Tet10 must reproduce the analytic small-deflection cantilever at slender proportions; \
         worst agreement at aspect >= 20 was {worst_slender:.4} (1.0 = exact). Absolute flex is \
         the product for a bending part, and nothing else in this repo validates it"
    );
}
