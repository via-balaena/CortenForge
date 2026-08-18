//! Does the solver's deformed-configuration validity check miss folds? — the
//! pilot that decided it, kept as the evidence the gate now rests on.
//!
//! `CpuNewtonSolver::check_orientation` is the gate that stops a folded element
//! reaching `first_piola`. It **used to sample five values**:
//!
//! - `det F` at the four Gauss points, where the constitutive model is
//!   evaluated, and
//! - the **corner-block affine** `det F`, from the four corners only.
//!
//! The rest-configuration meshers used four, then eight, and both were measured to
//! miss real folds — 18 on the conformed disc, then 5 more on the lofted one, the
//! worst inverted past its own straight volume. This asked whether the same gap
//! existed in the deformed configuration, where the consequence is worse: the
//! solver integrates over the folded element and takes `|det|`, silently.
//!
//! ✅ **It does, and the gate was converted.** `check_orientation` now certifies
//! `det J_def > 0` over the whole element and pairs it with a rest certificate taken
//! once at construction. `five_point_says_healthy` below is therefore the rule the
//! gate *used* to apply, kept as the comparator these measurements are stated
//! against — not a description of what ships.
//!
//! ⚠ An earlier revision of this header said the gate "runs on every Newton
//! iteration of every simulation". It does not: it runs at **two step boundaries**
//! per step (`newton.rs` — the incoming state and the converged one), while assembly
//! and factorization run per iteration. That mistake is what made the cost look
//! prohibitive. Measured on whole steps
//! (`what_one_step_costs_at_two_resolutions`), certifying it costs **~1 % of a
//! step** — 3 interleaved before/after pairs at each of two resolutions, the
//! certified side dearer in all 6.
//!
//! ⚠ Interleave the pairs. Run three of one condition and then three of the other
//! and machine drift is fully confounded with the change: done that way, the same
//! measurement read +4.3 % on the small mesh and **−2.0 % on the large one**, which
//! is not a cost profile, it is a clock.
//!
//! # Why the existing primitive applies unchanged
//!
//! `F(ξ) = J_def(ξ) · J_rest(ξ)⁻¹`, so `det F = det J_def / det J_rest`, and
//! `J_def(ξ) = x_defᵀ ∇N(ξ)` has **affine** entries exactly as `J_rest` does —
//! its determinant is a cubic in `ξ`. Since `det J_rest > 0` is now certified
//! over the whole element (`element::validity`), it follows that
//!
//! ```text
//!     det F > 0 everywhere   ⟺   det J_def > 0 everywhere
//! ```
//!
//! so `certify_rest(x_deformed, ValidityBar::Positive)` answers the deformed
//! question with **no new mathematics**. ⚠ An earlier note claimed the Bernstein
//! bound "does not transfer because F is not affine". `F` indeed is not — it is
//! a ratio — but the bound is never applied to `F`. It is applied to
//! `det J_def`, which is a cubic like any other.
//!
//! # What is measured
//!
//! 1. How often the five-point check calls an element healthy that is provably
//!    folded — on random deformed configurations of increasing severity.
//! 2. The same on a real curved mesh (the canonical layered sphere) driven
//!    through synthetic deformation.
//! 3. What certification costs against the five-point check, per element.
//!
//! # ⚠⚠ SCOPE: both real solves here are `NullContact`
//!
//! Contact is the regime where element inversion is most likely — a rigid
//! indenter drives sharp *local* compression, unlike the smooth global fields
//! bending and flexion produce — and **it is not covered here**.
//!
//! It was attempted and abandoned rather than fudged. A Tet10 IPC indentation
//! built on `tet10_indentation_demand1`'s own parameters stalls Armijo at ~6 %
//! of the target depth, and that gate's Tet10 arm is itself `#[ignore]`d as a
//! "deliberately-run one-shot measurement" (~60 min, never in CI) — so driving
//! one deep is a project, not a spot check. The tempting shortcut, widening the
//! IPC barrier band until it converges, **softens the contact and so makes
//! folds less likely** — it would weaken the very thing the census looks for
//! and produce a reassuring zero that means nothing.
//!
//! ⇒ Read the conclusion as **"no contact-free solve enters the blind spot"**.
//! Whether a contact solve does remains open — but it is no longer load-bearing:
//! the deferral it was blocking rested on a cost that turned out to be ~1 % of a
//! step, so the gate was certified without waiting for it. The contact census is
//! still worth running, as a measurement of how often the old rule was wrong in the
//! regime most likely to fold, rather than as the thing that decides the design.

#![allow(
    // Element/vertex counts index the `Mesh` trait's `u32` id space.
    clippy::cast_possible_truncation,
    // Population counts are exact well below 2^53; printed ratios are report
    // lines, not thresholds.
    clippy::cast_precision_loss,
    // A meshing failure surfaces as a test panic, which is the intent.
    clippy::expect_used
)]

use std::time::Instant;

/// Timing repetitions per method per round, and timed rounds after one warm-up.
const REPS: usize = 200;
const ROUNDS: usize = 3;

use nalgebra::SMatrix;
use sim_ml_chassis::Tensor;
use sim_soft::element::{Element, RestValidity, Tet10, ValidityBar, certify_rest};
use sim_soft::mesh::HandBuiltTetMesh;
use sim_soft::solver::CpuNewtonSolver;
use sim_soft::{
    BoundaryConditions, CpuTet10NHSolver, DifferenceSdf, LAYERED_SPHERE_CONFORM_QUALITY_FLOOR,
    LAYERED_SPHERE_R_CAVITY, LAYERED_SPHERE_R_OUTER, LoadAxis, MaterialField, Mesh, NullContact,
    SoftScene, Solver, SolverConfig, SphereSdf, Tet10Mesh, Vec3, VertexId,
    pick_vertices_by_predicate,
};

/// Deterministic `SplitMix64`.
struct Rng(u64);

impl Rng {
    const fn bits(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    #[allow(clippy::cast_precision_loss)] // >>11 leaves 53 bits: exact in f64.
    fn unit(&mut self) -> f64 {
        (self.bits() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn sym(&mut self) -> f64 {
        2.0f64.mul_add(self.unit(), -1.0)
    }
    fn vec(&mut self) -> Vec3 {
        Vec3::new(self.sym(), self.sym(), self.sym())
    }
}

/// The affine determinant of a 4-corner block — the quantity
/// `check_orientation`'s part (b) forms, up to the rest-frame factor that
/// cancels in the sign.
fn corner_block_det(x: &SMatrix<f64, 10, 3>) -> f64 {
    SMatrix::<f64, 3, 3>::from_fn(|r, c| x[(c + 1, r)] - x[(0, r)]).determinant()
}

/// **The solver's check, replicated exactly**: the sign verdict at the four
/// Gauss points plus the corner block.
///
/// `det F = det J_def / det J_rest` and `det J_rest > 0` is certified, so the
/// sign of `det F` is the sign of `det J_def` — which is what this reads. The
/// corner block is compared in the same way.
fn five_point_says_healthy(deformed: &SMatrix<f64, 10, 3>, rest: &SMatrix<f64, 10, 3>) -> bool {
    let gauss_ok = Tet10
        .rest_jacobian_dets(deformed)
        .iter()
        .all(|d| d.is_finite() && *d > 0.0);
    let corner = corner_block_det(deformed) / corner_block_det(rest);
    gauss_ok && corner.is_finite() && corner > 0.0
}

/// A rest element and a deformed image of it.
///
/// The rest mesh is a gently curved Tet10 (certifiably valid); the deformation
/// is a random field of increasing magnitude applied to its nodes, which is how
/// a solve reaches configurations that approach inversion.
fn rest_and_deformed(rng: &mut Rng, deform: f64) -> (SMatrix<f64, 10, 3>, SMatrix<f64, 10, 3>) {
    let base = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.5, 0.866, 0.0),
        Vec3::new(0.5, 0.289, 0.816),
    ];
    let corners: [Vec3; 4] = std::array::from_fn(|i| base[i] + rng.vec() * 0.08);
    let mut rest = [Vec3::zeros(); 10];
    rest[..4].copy_from_slice(&corners);
    for (i, &(a, b)) in sim_soft::element::TET10_EDGE_NODES.iter().enumerate() {
        rest[4 + i] = (corners[a] + corners[b]) * 0.5 + rng.vec() * 0.04;
    }
    let rest_m = SMatrix::<f64, 10, 3>::from_fn(|a, k| rest[a][k]);

    // The deformed image: every node displaced, midsides more than corners —
    // which is what bending does, and what the Gauss points are least able to
    // see.
    let mut def = rest;
    for (a, node) in def.iter_mut().enumerate() {
        let weight = if a < 4 { 0.35 } else { 1.0 };
        *node += rng.vec() * (deform * weight);
    }
    let def_m = SMatrix::<f64, 10, 3>::from_fn(|a, k| def[a][k]);
    (rest_m, def_m)
}

#[test]
fn how_much_does_the_five_point_deformed_check_miss() {
    println!("\n=== DEFORMED validity: five sampled points vs a proof ===");
    println!(
        "{:>8} {:>10} {:>10} {:>12} {:>14}",
        "deform", "certified", "folded", "undetermined", "5-pt MISSES"
    );

    let mut total_missed = 0usize;
    for level in 1..=7u32 {
        let deform = 0.04 * f64::from(level);
        let mut rng = Rng(0xDEF0_0000 + u64::from(level));
        let (mut certified, mut folded, mut undetermined) = (0usize, 0usize, 0usize);
        let mut missed = 0usize;

        for _ in 0..3000 {
            let (rest, def) = rest_and_deformed(&mut rng, deform);
            // Only elements whose REST state is sound are meaningful here — the
            // solver never starts from an invalid rest mesh (that is what the
            // mesher work certified).
            if !certify_rest(&rest, ValidityBar::Positive).is_certified() {
                continue;
            }
            let says_healthy = five_point_says_healthy(&def, &rest);
            match certify_rest(&def, ValidityBar::Positive) {
                RestValidity::Certified { .. } => certified += 1,
                RestValidity::Violated { .. } => {
                    folded += 1;
                    if says_healthy {
                        missed += 1;
                    }
                }
                RestValidity::Undetermined => undetermined += 1,
            }
        }
        total_missed += missed;
        let rate = if folded == 0 {
            0.0
        } else {
            100.0 * missed as f64 / folded as f64
        };
        println!(
            "{deform:>8.2} {certified:>10} {folded:>10} {undetermined:>12} {missed:>6} ({rate:>5.1}%)"
        );
    }
    println!(
        "  '5-pt MISSES' = elements PROVEN folded in the deformed configuration that the\n  \
         solver's four-Gauss-plus-corner-block check calls healthy. Every one of these would\n  \
         reach `first_piola` and be integrated with |det F|."
    );
    assert!(
        total_missed > 0,
        "if the five-point check misses nothing across this population, the deformed gate has \
         no gap and this pilot's premise is wrong — which is a finding, not a pass"
    );
}

#[test]
fn deformed_certification_cost_against_the_five_point_check() {
    let mut rng = Rng(0xDEF0_C057);
    let pop: Vec<(SMatrix<f64, 10, 3>, SMatrix<f64, 10, 3>)> = (0..1024)
        .map(|_| rest_and_deformed(&mut rng, 0.12))
        .collect();
    let mut best = [f64::INFINITY; 2];
    let mut sink = 0.0f64;
    for round in 0..=ROUNDS {
        let t = Instant::now();
        for _ in 0..REPS {
            for (rest, def) in &pop {
                sink += f64::from(five_point_says_healthy(
                    std::hint::black_box(def),
                    std::hint::black_box(rest),
                ));
            }
        }
        let five = t.elapsed().as_secs_f64();

        let t = Instant::now();
        for _ in 0..REPS {
            for (_, def) in &pop {
                sink += f64::from(
                    certify_rest(std::hint::black_box(def), ValidityBar::Positive).is_certified(),
                );
            }
        }
        let cert = t.elapsed().as_secs_f64();

        if round > 0 {
            let n = (REPS * pop.len()) as f64;
            best[0] = best[0].min(five / n);
            best[1] = best[1].min(cert / n);
        }
    }
    println!("\n=== DEFORMED check cost per element (ns) ===");
    println!("  5-point (Gauss + corner block) : {:8.1}", best[0] * 1e9);
    println!(
        "  certified (whole element)      : {:8.1}   ({:.2}x)",
        best[1] * 1e9,
        best[1] / best[0]
    );
    println!("  (sink {sink:.3e})");
}

// ---------------------------------------------------------------------------
// Real geometry
// ---------------------------------------------------------------------------

const MU: f64 = 2.0e5;
const NU: f64 = 0.4;
const PRESSURE: f64 = 5.0e3;
const CELL: f64 = 0.04;

fn body_sdf() -> DifferenceSdf {
    DifferenceSdf::new(
        Box::new(SphereSdf {
            radius: LAYERED_SPHERE_R_OUTER,
        }),
        Box::new(SphereSdf {
            radius: LAYERED_SPHERE_R_CAVITY,
        }),
    )
}

fn conformed_sphere() -> Tet10Mesh {
    let (mesh4, _bc, _initial, _theta) = SoftScene::layered_silicone_sphere(
        MaterialField::uniform(MU, 2.0 * MU * NU / (1.0 - 2.0 * NU)),
        CELL,
        PRESSURE,
    )
    .expect("layered_silicone_sphere meshes at the canonical cell size");
    Tet10Mesh::from_tet4(&mesh4)
        .with_sdf_projected_boundary(&body_sdf(), LAYERED_SPHERE_CONFORM_QUALITY_FLOOR)
}

fn element_nodes(mesh: &Tet10Mesh, t: u32) -> SMatrix<f64, 10, 3> {
    let corners = mesh.tet_vertices(t);
    let mids = mesh
        .tet_midside_nodes(t)
        .expect("Tet10Mesh surfaces the midside channel");
    let mut nodes = [Vec3::zeros(); 10];
    for (a, &c) in corners.iter().enumerate() {
        nodes[a] = mesh.positions()[c as usize];
    }
    for (i, &m) in mids.iter().enumerate() {
        nodes[4 + i] = mesh.positions()[m as usize];
    }
    SMatrix::<f64, 10, 3>::from_fn(|a, k| nodes[a][k])
}

/// The same question on a real curved mesh, under a deformation field rather
/// than random node noise: a radial squash of increasing severity, which is the
/// shape an internal-pressure or indentation solve drives.
#[test]
fn the_five_point_check_on_a_real_curved_mesh_under_deformation() {
    let mesh = conformed_sphere();
    let n = mesh.n_tets();
    println!("\n=== CANONICAL SPHERE, {n} elements, radial squash ===");
    println!(
        "{:>8} {:>10} {:>10} {:>12} {:>14}",
        "squash", "certified", "folded", "undetermined", "5-pt MISSES"
    );

    let rest: Vec<SMatrix<f64, 10, 3>> = (0..n as u32).map(|t| element_nodes(&mesh, t)).collect();
    let (mut total_folded, mut total_undetermined) = (0usize, 0usize);

    for level in 1..=6u32 {
        // Squash along z, expand in xy — volume-preserving-ish, and it drives
        // the boundary elements (the curved ones) hardest.
        let s = 0.12 * f64::from(level);
        let (mut certified, mut folded, mut undetermined, mut missed) = (0, 0, 0, 0usize);
        for r in &rest {
            let def = SMatrix::<f64, 10, 3>::from_fn(|a, k| {
                let v = r[(a, k)];
                match k {
                    2 => v * (1.0 - s),
                    _ => v * (1.0 + 0.5 * s),
                }
            });
            let says_healthy = five_point_says_healthy(&def, r);
            match certify_rest(&def, ValidityBar::Positive) {
                RestValidity::Certified { .. } => certified += 1,
                RestValidity::Violated { .. } => {
                    folded += 1;
                    missed += usize::from(says_healthy);
                }
                RestValidity::Undetermined => undetermined += 1,
            }
        }
        total_folded += folded;
        total_undetermined += undetermined;
        println!("{s:>8.2} {certified:>10} {folded:>10} {undetermined:>12} {missed:>14}");
    }
    println!(
        "  A pure affine squash cannot fold a valid element, so zeros here are EXPECTED and\n  \
         are the control: they say the instrument does not manufacture folds. The random\n  \
         population above is where a genuine gap would show."
    );

    // ⚠ THE CONTROL, asserted. A positive-determinant affine map cannot change
    // any element's orientation, so a fold reported here would mean the CENSUS
    // manufactures them — and every zero in the sibling tests would be
    // worthless. This is the assert that makes those zeros mean something.
    assert_eq!(
        (total_folded, total_undetermined),
        (0, 0),
        "an affine squash cannot fold a valid element; {total_folded} folded and \
         {total_undetermined} undetermined means the census itself is wrong, not the mesh"
    );
}

// ---------------------------------------------------------------------------
// A REAL SOLVE
// ---------------------------------------------------------------------------

const MU_BEAM: f64 = 1.0e5;
const NU_BEAM: f64 = 0.45;
const LENGTH: f64 = 0.5;
const BREADTH: f64 = 0.1;
const HEIGHT: f64 = 0.1;
const STATIC_DT: f64 = 1.0;

/// The cantilever bilayer beam, its clamped band and its tip band — the same
/// scene `tet10_bending_locking` drives, which is the canonical Tet10 bending
/// fixture in this crate.
fn cantilever_scene() -> (Tet10Mesh, Vec<VertexId>, Vec<VertexId>) {
    cantilever_scene_at(8, 2, 2)
}

/// [`cantilever_scene`] at an arbitrary cell resolution, so the cost arm can ask
/// how the gate's share moves with element count.
fn cantilever_scene_at(
    nx: usize,
    ny: usize,
    nz: usize,
) -> (Tet10Mesh, Vec<VertexId>, Vec<VertexId>) {
    let field = MaterialField::uniform(MU_BEAM, 2.0 * MU_BEAM * NU_BEAM / (1.0 - 2.0 * NU_BEAM));
    let tet4 =
        HandBuiltTetMesh::cantilever_bilayer_beam(nx, ny, nz, LENGTH, BREADTH, HEIGHT, &field);
    let tet10 = Tet10Mesh::from_tet4(&tet4);
    let pinned: Vec<VertexId> = pick_vertices_by_predicate(&tet10, |p| p.x.abs() < 1e-9);
    let loaded: Vec<VertexId> = pick_vertices_by_predicate(&tet10, |p| (p.x - LENGTH).abs() < 1e-9);
    assert!(
        !pinned.is_empty() && !loaded.is_empty(),
        "the clamped and tip bands must be non-empty"
    );
    (tet10, pinned, loaded)
}

/// ★★ The measurement that decided whether the blind spot matters — and, now that
/// it has been acted on, the regression net that says certification did not cost a
/// real solve: a quasi-static **bending** solve, censused at the states the
/// solver's own gate examines.
///
/// ⚠⚠ **`total_folded == 0` can no longer fail for the reason it was written.**
/// The gate now certifies these same states, so a folded step boundary makes
/// `try_replay_step` return `Err` and the loop breaks before the census sees it.
/// That assert is kept as a cross-check that the census and the gate agree about
/// what the certificate says, not as a discovery — and it is not what makes this
/// test worth running.
///
/// ★ **`max_tip` is the live assertion.** Certification is strictly stricter than
/// the five-point rule, so the failure mode this test now guards is the opposite
/// one: a gate so strict that real bending stops converging. The beam reaches
/// 136 % of its own length in tip deflection; if certification ever refused a step
/// before that, the ramp would break early and `max_tip` would fall short.
///
/// `check_validity_at_step_start` runs on the incoming state at each step
/// boundary, so censusing every step boundary examines exactly the states the
/// gate does — no solver instrumentation required, and no risk of measuring
/// something the gate never sees.
///
/// Bending is the case that matters. The corner block is affine and the Gauss
/// points are interior, so **midside-driven** folding is what the check cannot
/// see, and bending is precisely what moves midsides relative to their corners.
/// It is also the reason Tet10 exists at all.
///
/// The tip load is walked up so the beam passes through increasing deflection
/// rather than jumping to one state: fold onset is what the random population
/// showed the check missing 100 % of, so the interesting region is the approach,
/// not the extreme.
#[test]
fn a_real_bending_solve_censused_at_every_step_boundary() {
    let (tet10, pinned, loaded) = cantilever_scene();

    let rest: Vec<SMatrix<f64, 10, 3>> = (0..tet10.n_tets() as u32)
        .map(|t| element_nodes(&tet10, t))
        .collect();
    let n_tets = rest.len();

    let positions = tet10.positions();
    let n_dof = 3 * positions.len();
    let mut x_flat = vec![0.0; n_dof];
    for (v, p) in positions.iter().enumerate() {
        x_flat[3 * v] = p.x;
        x_flat[3 * v + 1] = p.y;
        x_flat[3 * v + 2] = p.z;
    }

    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: Vec::new(),
        loaded_vertices: loaded.iter().map(|&v| (v, LoadAxis::AxisZ)).collect(),
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = 500;
    cfg.tol = 1e-8;
    let solver: CpuTet10NHSolver<Tet10Mesh> =
        CpuNewtonSolver::new(Tet10, tet10.clone(), NullContact, cfg, bc);

    println!(
        "\n=== REAL BENDING SOLVE — {n_tets} Tet10 elements, censused at every step boundary ==="
    );
    println!(
        "{:>10} {:>10} {:>10} {:>9} {:>13} {:>14}",
        "tip load", "tip defl", "certified", "folded", "undetermined", "5-pt MISSES"
    );

    let v_prev = Tensor::zeros(&[n_dof]);
    let mut x_prev = Tensor::from_slice(&x_flat, &[n_dof]);
    let mut total_missed = 0usize;
    let mut total_folded = 0usize;
    let mut max_tip = 0.0f64;

    // Walked far past any sane operating point, until the solver refuses: the
    // question is whether folds appear BEFORE the gate fires, not whether a
    // reasonable load is safe.
    for level in 1..=40u32 {
        let load = 2.0 * f64::from(level) * f64::from(level).max(1.0) / 4.0;
        let theta = Tensor::from_slice(&[load / loaded.len() as f64], &[1]);
        let Ok(step) = solver.try_replay_step(&x_prev, &v_prev, &theta, cfg.dt) else {
            println!("{load:>10.1}   solver refused this step (validity or convergence) — stop");
            break;
        };

        // The step-boundary state: exactly what `check_validity_at_step_start`
        // would be handed on the NEXT step.
        let x = &step.x_final;
        let mut tip = 0.0f64;
        for &v in &loaded {
            tip = tip.max((x[3 * v as usize + 2] - x_flat[3 * v as usize + 2]).abs());
        }

        let (mut certified, mut folded, mut undetermined, mut missed) = (0, 0, 0, 0usize);
        for (t, r) in rest.iter().enumerate() {
            let nodes = tet10
                .tet_midside_nodes(t as u32)
                .expect("Tet10 midside channel");
            let corners = tet10.tet_vertices(t as u32);
            let ids: [VertexId; 10] = [
                corners[0], corners[1], corners[2], corners[3], nodes[0], nodes[1], nodes[2],
                nodes[3], nodes[4], nodes[5],
            ];
            let def = SMatrix::<f64, 10, 3>::from_fn(|a, k| x[3 * ids[a] as usize + k]);
            let says_healthy = five_point_says_healthy(&def, r);
            match certify_rest(&def, ValidityBar::Positive) {
                RestValidity::Certified { .. } => certified += 1,
                RestValidity::Violated { .. } => {
                    folded += 1;
                    missed += usize::from(says_healthy);
                }
                RestValidity::Undetermined => undetermined += 1,
            }
        }
        total_missed += missed;
        total_folded += folded;
        println!(
            "{load:>10.1} {tip:>10.4} {certified:>10} {folded:>9} {undetermined:>13} {missed:>14}"
        );
        max_tip = max_tip.max(tip);
        x_prev = Tensor::from_slice(x, &[n_dof]);
    }

    println!(
        "  Every 'MISS' is an element the solver's gate passes at a step boundary and then\n           integrates with |det F| for the whole of the next step."
    );
    println!("  totals: {total_folded} folded, {total_missed} of them invisible to the gate");

    // Cross-check, not a discovery: the gate certifies these very states, so it
    // would have refused the step before the census could see a fold. This asserts
    // the census and the gate read the same certificate — a disagreement would mean
    // one of them is not calling `certify_orientation` on the state it claims to.
    assert_eq!(
        total_folded, 0,
        "the census found {total_folded} folded elements at states the solver's own gate \
         certified ({total_missed} of them invisible to the five-point rule). The gate and \
         this census disagree, which means one of them is not reading the state it claims to"
    );
    // ★ THE LIVE ASSERTION. Certification is strictly stricter than what it
    // replaced, so the risk it introduces is refusing steps a real solve needs. If
    // the ramp above ever breaks early, this is what says so.
    assert!(
        max_tip > 0.5 * LENGTH,
        "the beam must reach a real deflection under the certified gate; max tip deflection \
         was {max_tip:.4} m on a {LENGTH} m beam. A short ramp here means certification is \
         refusing steps that the five-point rule allowed"
    );
}

// ---------------------------------------------------------------------------
// WHAT THE CONVERSION WOULD COST A CALLER
// ---------------------------------------------------------------------------

/// Untimed load levels walked before timing starts, then the level each timed
/// step advances to. A caller ramps; it does not jump to full load from rest.
const WARM_RAMP: [f64; 4] = [8.0, 32.0, 72.0, 128.0];
const TIMED_LOAD: f64 = 162.0;

/// Timed steps per round, and timed rounds after one untimed warm-up.
const TIMED_STEPS: usize = 3;

/// ★ The measurement that decides the conversion — whole steps, not the predicate.
///
/// [`deformed_certification_cost_against_the_five_point_check`] says the certificate
/// costs ~2.2x the five-point check **per element**. That ratio is not what a caller
/// pays, and reading it as though it were is how this deferral was originally argued
/// ("2.31x on the hot path"). The gate is not the step: `check_validity_at_step_start`
/// runs at **two step boundaries** — once on the incoming state, once on the converged
/// one (`newton.rs`) — while assembly, factorization and every Armijo backtrack run
/// per Newton iteration. So this arm times `try_replay_step` itself. Run it before and
/// after a change to the gate and the difference is what a caller actually pays, with
/// no arithmetic in between and no assumption about how often the gate is called.
///
/// ⚠ The small beam is kept deliberately. The gate is **linear** in element count while
/// the factorization is superlinear, so the gate's share is largest on the *smallest*
/// mesh — refining makes a gate change look cheaper, not dearer. Reporting only the
/// refined row would flatter the conversion.
///
/// ⚠ Every timed step is an INCREMENT from a pre-solved neighbour, which is what a
/// simulation does. An earlier draft timed a jump from rest to full load instead: that
/// is hundreds of Newton iterations of a kind no caller runs, it charged the step
/// **2180 ms** against this form's 42 ms — a 51x self-inflicted inflation that made the
/// fixture unrunnable rather than conservative.
///
/// `#[ignore]`: a deliberately-run measurement, not a gate. Wall-clock thresholds in CI
/// are noise detectors, and this arm's whole purpose is to be read by a human deciding
/// whether a gate change is affordable. Run it with `--ignored --nocapture`, in
/// **release** — the debug numbers measure the debug build, not the engine.
#[test]
#[ignore = "deliberately-run cost measurement (~40 s, release); not a CI gate"]
fn what_one_step_costs_at_two_resolutions() {
    println!("\n=== STEP COST (release) — the denominator a gate change is a share of ===");
    println!(
        "{:>10} {:>10} {:>12} {:>16}",
        "elements", "DOFs", "ms / step", "us / element"
    );
    for (nx, ny, nz) in [(8usize, 2usize, 2usize), (12, 3, 4)] {
        let (tet10, pinned, loaded) = cantilever_scene_at(nx, ny, nz);
        let n_tets = tet10.n_tets();

        let positions = tet10.positions();
        let n_dof = 3 * positions.len();
        let mut x_flat = vec![0.0; n_dof];
        for (v, p) in positions.iter().enumerate() {
            x_flat[3 * v] = p.x;
            x_flat[3 * v + 1] = p.y;
            x_flat[3 * v + 2] = p.z;
        }

        let n_loaded = loaded.len() as f64;
        let bc = BoundaryConditions {
            pinned_vertices: pinned,
            roller_vertices: Vec::new(),
            loaded_vertices: loaded.iter().map(|&v| (v, LoadAxis::AxisZ)).collect(),
        };
        let mut cfg = SolverConfig::skeleton();
        cfg.dt = STATIC_DT;
        cfg.max_newton_iter = 500;
        cfg.tol = 1e-8;
        let solver: CpuTet10NHSolver<Tet10Mesh> =
            CpuNewtonSolver::new(Tet10, tet10.clone(), NullContact, cfg, bc);

        // Walk to a working deflection, untimed — the timed step then advances
        // from a converged neighbour, so it costs what a simulation's steps cost.
        let v_prev = Tensor::zeros(&[n_dof]);
        let mut x_prev = Tensor::from_slice(&x_flat, &[n_dof]);
        for load in WARM_RAMP {
            let theta = Tensor::from_slice(&[load / n_loaded], &[1]);
            let step = solver
                .try_replay_step(&x_prev, &v_prev, &theta, cfg.dt)
                .expect("the warm ramp must converge for the timings below to mean anything");
            x_prev = Tensor::from_slice(&step.x_final, &[n_dof]);
        }

        let theta = Tensor::from_slice(&[TIMED_LOAD / n_loaded], &[1]);
        let warm = solver
            .try_replay_step(&x_prev, &v_prev, &theta, cfg.dt)
            .expect("the timed increment must converge");
        let mut tip = 0.0f64;
        for (v, p) in positions.iter().enumerate() {
            tip = tip.max((warm.x_final[3 * v + 2] - p.z).abs());
        }

        let mut best = f64::INFINITY;
        for _ in 0..ROUNDS {
            let t = Instant::now();
            for _ in 0..TIMED_STEPS {
                let step = solver
                    .try_replay_step(&x_prev, &v_prev, &theta, cfg.dt)
                    .expect("the timed increment converged during warm-up");
                std::hint::black_box(step.x_final[0]);
            }
            best = best.min(t.elapsed().as_secs_f64() / TIMED_STEPS as f64);
        }
        println!(
            "{n_tets:>10} {n_dof:>10} {:>12.3} {:>16.2}",
            best * 1e3,
            best * 1e6 / n_tets as f64
        );
        // ⚠ Non-vacuity: a step that barely moves the beam is not the step whose
        // cost is being reported. The ramp arm above reaches 0.6 m on this beam.
        assert!(
            tip > 0.3 * LENGTH,
            "the timed step must land on a genuinely deflected state; tip was {tip:.4} m"
        );
    }
    println!(
        "  Read `deformed_certification_cost_against_the_five_point_check`'s per-element\n  \
         figures against THIS column, not against each other. The gate runs twice per\n  \
         step, so its share is 2 x (per-element ns) x elements / (ms per step)."
    );
}
