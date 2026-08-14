//! ★★★ **Solid tets lock in slender bending — the measured ceiling.**
//!
//! A Tet10 cantilever at 20:1 deflects to **a quarter** of the analytic
//! small-deflection answer, and reports success while doing it. This is not a
//! resolution problem and not a tolerance problem: it is tetrahedral locking in
//! bending, and it caps every slender bending part the engine could simulate —
//! a hockey stick, a leaf spring, an exo strut, a rib.
//!
//! # What is asserted here
//!
//! Three gates, each pinning one half of the conclusion:
//!
//! 1. [`a_slender_tet10_cantilever_lands_near_a_quarter_of_analytic`] — the
//!    magnitude of the error, against a closed-form target.
//! 2. [`neither_mesh_direction_recovers_the_slender_deflection`] — it is
//!    mesh-converged **both** ways, so refinement is not the missing piece.
//! 3. [`element_order_does_not_rescue_slender_bending`] — the Tet10/Tet4 gain is
//!    flat and does not grow with slenderness, so p-refinement is not the path.
//!
//! Together they say: the answer is a different element **family** (hexahedra),
//! not a finer mesh, not a cubic tet, and not a looser tolerance.
//!
//! # ⚠ These gates go RED when the engine gets BETTER
//!
//! They characterise a limit rather than requiring one. The day a
//! slender-capable element lands, gate 1 stops reading ~0.25 and reds. **That is
//! the signal, not a regression** — update the band and record the new floor.
//! Written this way deliberately: the alternative, `assert!(ratio > 0.85)`,
//! encodes a requirement the engine provably does not meet and can only ever be
//! a red test nobody can act on.
//!
//! # Why the analytic target is trustworthy
//!
//! Every measurement is driven in the **linear regime** (`δ/L ≈ 1e-3`) where
//! `δ = P L³ / (3 E I)` is exact, with `I = h⁴/12` and `E = 2μ(1+ν)`. The two
//! corrections that could invalidate it are bounded rather than assumed away:
//! shear (Timoshenko) adds `(1+ν)h²/(2kL²)` relative — **0.05 %** at 40:1 —
//! and large-deflection stiffening only appears above `δ/L ≈ 0.1`, two orders
//! above what is driven. ⚠ Deliberately **not** the elastica: those values would
//! have come from memory, and ground truth that is half-remembered validates
//! nothing.
//!
//! # ⚠ What this file no longer tries to answer
//!
//! It began as a VR frame-budget pilot — "can a flexing Tet10 stick fit 11.1 ms
//! at 90 Hz". That question is **blocked behind this one** and its scaffolding
//! has been removed: a milliseconds-per-step figure for a shaft that bends four
//! times too little is not a useful number. The budget work resumes on an
//! element that passes gate 1.

#![allow(
    // Element counts and loop bounds are exact well below 2^53; printed ratios
    // are report lines, not thresholds.
    clippy::cast_precision_loss
)]

use sim_ml_chassis::Tensor;
use sim_soft::element::{Tet4, Tet10};
use sim_soft::mesh::HandBuiltTetMesh;
use sim_soft::solver::CpuNewtonSolver;
use sim_soft::{
    BoundaryConditions, CpuTet4NHSolver, CpuTet10NHSolver, LoadAxis, MaterialField, Mesh,
    NullContact, Solver, SolverConfig, Tet10Mesh, VertexId, pick_vertices_by_predicate,
};

/// Quasi-static: one step, no dynamics.
const STATIC_DT: f64 = 1.0;

/// Beam length (m). Slenderness is varied by shrinking the section, not this.
const L: f64 = 1.5;

/// Driven tip deflection as a fraction of length — small enough that
/// Euler–Bernoulli is exact to well under a percent.
const TARGET_RATIO: f64 = 1.0e-3;

const MU: f64 = 1.0e5;
const NU: f64 = 0.35;

/// The tip load that lands a cantilever of section `h` on [`TARGET_RATIO`].
///
/// Inverts `δ = P L³ / (3 E I)`. Loading to a fixed *deflection ratio* rather
/// than a fixed force is what keeps rows comparable: without it a row could
/// differ from its neighbour both in slenderness and in how hard it was pushed.
fn load_for(h: f64) -> f64 {
    let e = 2.0 * MU * (1.0 + NU);
    TARGET_RATIO * L * 3.0 * e * (h.powi(4) / 12.0) / L.powi(3)
}

/// Everything one cantilever solve needs, derived from its section.
///
/// ⚠ The tolerance is relative to the load, and the load here is tiny (~6e-5 N):
/// a `1e-9` relative tolerance would ask for a ~1e-14 residual, which is machine
/// epsilon and unreachable. A tolerance is only meaningful once its ABSOLUTE
/// scale has been checked against the problem.
fn rig<M: Mesh>(mesh: &M, load: f64) -> (Vec<f64>, Vec<f64>, BoundaryConditions, SolverConfig) {
    let n_dof = 3 * mesh.n_vertices();
    let pinned: Vec<VertexId> = pick_vertices_by_predicate(mesh, |p| p.x.abs() < 1e-9);
    let loaded: Vec<VertexId> = pick_vertices_by_predicate(mesh, |p| (p.x - L).abs() < 1e-9);
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

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = 500;
    cfg.tol = 1e-6 * load / (loaded.len() as f64).sqrt();

    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: Vec::new(),
        loaded_vertices: loaded.iter().map(|&v| (v, LoadAxis::AxisZ)).collect(),
    };
    (x_flat, rest_z, bc, cfg)
}

/// Peak tip deflection of the loaded band, relative to rest.
fn tip_of(x_final: &[f64], loaded: &[(VertexId, LoadAxis)], rest_z: &[f64]) -> f64 {
    loaded
        .iter()
        .zip(rest_z)
        .map(|(&(v, _), &z0)| (x_final[3 * v as usize + 2] - z0).abs())
        .fold(0.0f64, f64::max)
}

/// `(fem/analytic, tip_deflection_m, converged)` for a Tet10 cantilever.
fn tet10_ratio(aspect: f64, nx: usize, nz: usize) -> (f64, f64, bool) {
    let h = L / aspect;
    let load = load_for(h);
    let lambda = 2.0 * MU * NU / (1.0 - 2.0 * NU);
    let field = MaterialField::uniform(MU, lambda);
    let tet4 = HandBuiltTetMesh::cantilever_bilayer_beam(nx, 2, nz, L, h, h, &field);
    let mesh = Tet10Mesh::from_tet4(&tet4);
    let n_dof = 3 * mesh.n_vertices();
    let (x_flat, rest_z, bc, cfg) = rig(&mesh, load);
    let loaded = bc.loaded_vertices.clone();
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
            let tip = tip_of(&step.x_final, &loaded, &rest_z);
            (tip / (TARGET_RATIO * L), tip, true)
        })
}

/// `(fem/analytic, tip_deflection_m, converged)` for a Tet4 cantilever.
///
/// Shares the mesh, load, tolerance and analytic target with [`tet10_ratio`], so
/// the only difference between the two is the element.
fn tet4_ratio(aspect: f64, nx: usize) -> (f64, f64, bool) {
    let h = L / aspect;
    let load = load_for(h);
    let lambda = 2.0 * MU * NU / (1.0 - 2.0 * NU);
    let field = MaterialField::uniform(MU, lambda);
    let mesh = HandBuiltTetMesh::cantilever_bilayer_beam(nx, 2, 2, L, h, h, &field);
    let n_dof = 3 * mesh.n_vertices();
    let (x_flat, rest_z, bc, cfg) = rig(&mesh, load);
    let loaded = bc.loaded_vertices.clone();
    let solver: CpuTet4NHSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);

    let theta = Tensor::from_slice(&[load / loaded.len() as f64], &[1]);
    solver
        .try_replay_step(
            &Tensor::from_slice(&x_flat, &[n_dof]),
            &Tensor::zeros(&[n_dof]),
            &theta,
            cfg.dt,
        )
        .map_or((f64::NAN, f64::NAN, false), |step| {
            let tip = tip_of(&step.x_final, &loaded, &rest_z);
            (tip / (TARGET_RATIO * L), tip, true)
        })
}

/// ★★★ **The ceiling itself: a slender Tet10 cantilever lands near a QUARTER of
/// the analytic deflection.**
///
/// `tet10_bending_locking` already validates a *ratio* — Tet10 against Tet4, and
/// ν-sensitivity — at **5:1**. It never checked an **absolute** deflection
/// against a closed-form answer, and never at slenderness. The element was known
/// to be *better than Tet4*, which is true and is not the same as *correct*.
///
/// ⚠ Reds when the engine improves — see the module header. The band is wide
/// enough to absorb solver noise and narrow enough that a real fix trips it.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn a_slender_tet10_cantilever_lands_near_a_quarter_of_analytic() {
    println!("\n=== Tet10 vs analytic small-deflection cantilever ===");
    println!("  δ_analytic = P L³ / (3 E I), driven at δ/L ≈ 1e-3 so Euler–Bernoulli is exact.\n");
    println!(
        "{:>8} {:>6} {:>14} {:>12}",
        "aspect", "nz", "δ_fem (mm)", "fem/exact"
    );

    // ⚠ Counted, because a bound that only moves on success starts perfect: an
    // all-diverged table would sail through the asserts below reporting exact
    // agreement. That is the "gate that cannot fail for the reason it claims"
    // shape, and this gate passed exactly once before the counter existed.
    let mut slender_rows = 0usize;
    let mut worst = f64::INFINITY;
    let mut best = 0.0f64;
    for aspect in [5.0, 10.0, 20.0] {
        for nz in [2, 4] {
            let (ratio, tip, ok) = tet10_ratio(aspect, 8, nz);
            println!(
                "{aspect:>8.0} {nz:>6} {:>14.5} {:>12}",
                tip * 1e3,
                if ok {
                    format!("{ratio:.4}")
                } else {
                    "DIVERGED".to_string()
                }
            );
            assert!(
                ok,
                "aspect {aspect}:1 nz={nz} must converge to be evidence at all"
            );
            if aspect >= 20.0 {
                slender_rows += 1;
                worst = worst.min(ratio);
                best = best.max(ratio);
            }
        }
    }

    assert!(
        slender_rows > 0,
        "no slender row was measured, so the figures below are vacuous"
    );
    assert!(
        (0.15..0.40).contains(&worst) && (0.15..0.40).contains(&best),
        "expected the 20:1 Tet10 cantilever to sit near a QUARTER of analytic \
         (measured 0.2493 on 2026-08-13); got {worst:.4}..{best:.4}. If this rose \
         toward 1.0 a slender-capable element landed — that is the good outcome: \
         re-anchor this band and record the new floor. If it fell, something \
         regressed"
    );
}

/// ★★ **It is mesh-converged in BOTH directions**, so refinement is not the
/// missing piece.
///
/// ⚠⚠ This gate exists because a conclusion was nearly shipped without it. An
/// earlier sweep varied `nz` (thickness), saw no movement, and concluded solid
/// Tet10 shear-locks at slenderness. But `nx` was hardcoded at 8 throughout,
/// leaving **10:1 element aspect ratios** in every row — the exact shape that
/// locks. "Mesh does not fix it" was really "thickness refinement does not fix
/// it", a far weaker claim. Refining along the beam is what represents curvature
/// and what brings element aspect back toward 1:1, so it is swept here.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn neither_mesh_direction_recovers_the_slender_deflection() {
    println!("\n=== Mesh convergence at 20:1, both directions ===");
    println!(
        "{:>5} {:>5} {:>14} {:>12}",
        "nx", "nz", "elem aspect", "fem/exact"
    );

    let h = L / 20.0;
    // Tracked in the loop rather than collected, so the comparison below needs
    // no fallible indexing into a list that is a literal three lines up.
    let mut coarse = f64::NAN;
    let mut fine = f64::NAN;
    for (i, nx) in [8usize, 16, 32, 64].into_iter().enumerate() {
        let (ratio, _, ok) = tet10_ratio(20.0, nx, 2);
        let elem_aspect = (L / nx as f64) / (h / 2.0);
        println!("{nx:>5} {:>5} {elem_aspect:>14.2} {ratio:>12.4}", 2);
        assert!(ok, "nx={nx} must converge to be evidence");
        if i == 0 {
            coarse = ratio;
        }
        fine = ratio;
    }
    let (thick, _, ok_thick) = tet10_ratio(20.0, 64, 4);
    println!(
        "{:>5} {:>5} {:>14.2} {thick:>12.4}",
        64,
        4,
        (L / 64.0) / (h / 4.0)
    );
    assert!(ok_thick, "the through-thickness row must converge");

    let along_move = (fine - coarse).abs() / coarse;
    let thick_move = (thick - fine).abs() / fine;
    println!(
        "\n  nx 8→64 moves the answer {:.2} %, nz 2→4 moves it {:.2} % — converged both ways.",
        along_move * 100.0,
        thick_move * 100.0
    );

    // Element aspect goes 5.0 → 0.62 across this sweep. If badly-shaped elements
    // were the cause, THIS is where the answer would climb.
    assert!(
        along_move < 0.10,
        "refining 8× along the beam moved the answer {:.1} % — if length refinement now \
         recovers the deflection, the ceiling was element SHAPE, not the element, and \
         `a_slender_tet10_cantilever_lands_near_a_quarter_of_analytic` should have moved too",
        along_move * 100.0
    );
    assert!(
        thick_move < 0.10,
        "through-thickness refinement moved the answer {:.1} %",
        thick_move * 100.0
    );
    assert!(
        fine < 0.40,
        "a fully refined 20:1 shaft still must not reach the analytic answer; got {fine:.4}"
    );
}

/// ★★ **Order does not rescue it** — the discriminator that chose hexahedra.
///
/// Two fixes follow from two very different diagnoses, and only a measurement
/// separates them:
///
/// - **Tet4 much worse than Tet10, and the gap widening with slenderness** ⇒ the
///   error is dominated by element **order**, and p-refinement is a live path: a
///   cubic tet would continue the trend and the element family extends.
/// - **Tet4 ≈ Tet10, gain flat** ⇒ order does not rescue it, and the answer is a
///   different element **family** — hexahedra, what commercial FEM reaches for in
///   bending-dominated parts, and a correspondingly larger addition.
///
/// Measured here (`nx = 16`, both arms): **1.75× at 5:1, 1.72× at 10:1, 1.70× at
/// 20:1** — flat, and if anything narrowing. Extrapolating a constant 1.7× per
/// order from 0.2507: one more order reaches ~0.43, another ~0.72 — three-plus
/// orders to arrive at correct, and worse again at 40:1. ⇒ **Hexahedra.**
///
/// ⚠ The gain is **mesh-dependent**, so these figures belong to this test's grid
/// and not to the element in the abstract. An earlier sweep at `nx = 8` read
/// 1.69/1.54/1.37 and was recorded as "shrinks with slenderness"; at `nx = 16`
/// it is flat instead. The conclusion is the same either way — a gain that does
/// not *grow* cannot make p-refinement the cheaper path — which is why the
/// assertion below tests the trend rather than pinning the three numbers.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn element_order_does_not_rescue_slender_bending() {
    println!("\n=== Discriminator: does ELEMENT ORDER rescue slender bending? ===");
    println!("  same mesh, load, tolerance and analytic target — only the element differs.\n");
    println!(
        "{:>8} {:>12} {:>12} {:>12}",
        "aspect", "Tet4", "Tet10", "Tet10/Tet4"
    );

    let mut stubby_gain = f64::NAN;
    let mut slender_gain = f64::NAN;
    for (i, aspect) in [5.0, 10.0, 20.0].into_iter().enumerate() {
        let (r4, _, ok4) = tet4_ratio(aspect, 16);
        let (r10, _, ok10) = tet10_ratio(aspect, 16, 2);
        assert!(
            ok4 && ok10,
            "both arms must converge at {aspect}:1 to be a comparison"
        );
        let gain = r10 / r4;
        println!("{aspect:>8.0} {r4:>12.4} {r10:>12.4} {gain:>11.2}x");
        if i == 0 {
            stubby_gain = gain;
        }
        slender_gain = gain;
    }
    assert!(
        slender_gain < 2.0,
        "the Tet10/Tet4 gain at 20:1 was {slender_gain:.2}× — a large gain would mean order IS \
         doing the work and p-refinement is the path, which would overturn the choice of \
         hexahedra recorded in this module"
    );
    // 10 % of slack, not zero: measured 1.75× → 1.70×, and a bare `<=` would sit
    // 0.05 from flipping on a platform's last-bit differences. What has to be
    // refuted is a gain that GROWS — an order-dominated error shows 2×, 4×, not
    // a percent of drift.
    assert!(
        slender_gain <= stubby_gain * 1.10,
        "the order gain must not GROW with slenderness ({stubby_gain:.2}× at 5:1 vs \
         {slender_gain:.2}× at 20:1) — a growing gain is the signature of an order-dominated \
         error, and p-refinement would then be the cheaper fix than a new element family"
    );
}
