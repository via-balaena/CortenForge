//! **How coarse can a hockey stick be and still flex right — and what does that
//! cost per frame?**
//!
//! This is the fixture the sim-soft real-time ladder is missing. Everything
//! measured so far (`recon §2f–§2l`) sized reduction against an *indentation*
//! fixture at 5 202 and 18 750 free DOF. The stated real-time target is a
//! **hockey sim with realistic stick flex**, and §2a already brackets that
//! target on either side:
//!
//! ```text
//!    540 free DOF   0.64 ms / Newton iteration   -> 6 iters = 3.9 ms   FITS a game budget
//!  3 000 free DOF   8.4  ms / Newton iteration   -> ONE iter busts it
//! ```
//!
//! ⚠ **Those two come from §2a's `dt = 1/60` rows, not from its per-iteration
//! table**, and the distinction matters because looking §2a up finds a
//! *different* pair. Its per-iteration table reads `0.57` and `7.63`, measured
//! at `dt = 1e-3` with exactly 2 Newton iterations per step. The figures above
//! are `ms/frame ÷ iterations/step` on the same section's 60 Hz rows —
//! `7.93 / 12.3` and `202.6 / 24.2` — which is the quantity a game cares about,
//! since a game steps at the frame rate.
//!
//! So the whole reduced-order ladder is on this target's critical path **only
//! if a convincing stick needs more than ~1 000 free DOF**. That is a question
//! about *discretisation*, not about reduction, and it has never been asked
//! here. This file asks it.
//!
//! # The fixture, and where every number in it comes from
//!
//! A real senior shaft is a **hollow** composite tube. Modelling it as a solid
//! bar of the same outer section would be wrong twice over — too stiff, and far
//! too heavy. So the solid twin is **calibrated on the two quantities that set
//! bending**: `EI` (static flex) and mass per unit length (dynamics, since
//! `f₁ ∝ √(EI / m′L⁴)`). Match those and a solid rectangular beam is a faithful
//! Euler–Bernoulli twin of the real tube.
//!
//! | quantity | value | where from |
//! |---|---|---|
//! | free span `L` | `1.20 m` | shaft below the lower hand — the part that whips |
//! | width `b` (y) | `0.019 m` | senior shaft outer section, side-to-side |
//! | depth `h` (z) | `0.029 m` | senior shaft outer section, front-to-back = **the flex direction** |
//! | `EI` | `310 N·m²` | an 85-flex shaft: `85 lbf = 378 N` over a 1 m three-point span at 1 inch |
//! | `m′` | `0.25 kg/m` | ~400 g senior stick over ~1.6 m |
//!
//! ⚠ **`EI = 310` is one reading of the flex rating, not a standard.** "Flex" is
//! the force to deflect the shaft an inch, but the *span* is not standardised
//! across manufacturers; read as a `0.56 m` cantilever instead, the same rating
//! gives `EI ≈ 880`. That 2.8× spread does **not** move this file's verdict, and
//! that is asserted rather than asserted-away — see
//! [`the_discretisation_verdict_does_not_depend_on_stiffness`].
//!
//! Everything else is derived, so the fixture has two free inputs, not six:
//!
//! ```text
//!   I_solid = b·h³/12 = 3.862e-8 m⁴        E_eff = EI / I_solid = 8.03 GPa
//!   A       = b·h     = 5.51e-4 m²         ρ_eff = m′ / A       = 454 kg/m³
//! ```
//!
//! ★ `E_eff = 8.0 GPa` implies a real tube modulus of
//! `E_eff · I_solid / I_hollow ≈ 25 GPa` — taking a `1.2 mm` wall, i.e.
//! `I_hollow = 1.26e-8 m⁴` — low for unidirectional carbon
//! (`130+ GPa`), right for a real layup
//! carrying a lot of off-axis ply. The fixture lands where a stick actually is.
//!
//! # What is measured, and what is deliberately not
//!
//! **Measured here — discretisation error against a closed form.** Driven to
//! `δ/L = 1e-3`, where Euler–Bernoulli is exact: geometric nonlinearity is
//! negligible, and the Timoshenko shear correction at `L/h = 41` is `~0.06 %`.
//! Any departure from `1.0` is the *element and the mesh*, nothing else.
//!
//! **⛔ Not measured here — the regime the game actually runs.** A slapshot
//! deflects a stick `~100 mm`, i.e. `δ/L ≈ 0.08`, which is deeply
//! geometrically nonlinear and has no closed form. That needs a self-convergence
//! study against the finest mesh, and it is a separate rung. So is the dynamic
//! first bending mode. ★ The static ratio does **bound** the frequency error
//! without measuring it: a mesh reading `r` of analytic is `1/r` too stiff, so
//! its `f₁` is `1/√r` too high — `r = 0.90` is a `5.4 %` frequency error. That
//! bound is printed per row; it is arithmetic, not a measurement.
//!
//! # ⚠ Tet4 locks on slender geometry, and that is the point of the sweep
//!
//! A constant-strain Tet4 under-resolves bending, and `tet10_bending_locking`
//! measures it at `~46 %` of analytic on a **5:1** beam. This stick is **41:1**.
//! If locking forces the DOF count up, the answer is a better *element*, not a
//! reduced basis — so all three arms are swept together and compared **at equal
//! accuracy**, which is the only comparison that decides anything:
//!
//! - **Tet4** — cheapest per DOF, expected to lock.
//! - **Tet4 + F-bar** — a control, and expected to be a near-no-op. F-bar cures
//!   *volumetric* locking (`ν → 0.5`); this is a composite at `ν = 0.3` and the
//!   failure here is bending under-resolution, which F-bar does not touch.
//! - **Tet10** — quadratic, does not lock (`slender_bending_matches_analytic`
//!   recovers `0.97–0.995` at 20:1), but carries more nodes per cell —
//!   **`5.0–6.0×`, measured** by the sizing preview below on these grids, not
//!   the `~8×` a structured-grid estimate suggests.

// Loaded-vertex counts and grid indices cast to `f64` for the per-vertex load
// split and the analytic comparison — the FEM idiom shared with
// `slender_bending_matches_analytic.rs` and `fbar_locking.rs`, not a
// precision-sensitive path.
#![allow(clippy::cast_precision_loss)]
// `expect` on a value this file has just established to be present — a failure
// row that must carry a reason, a timing sample that cannot be NaN. The
// convention across this crate's tests (`contact_drop_rest.rs`,
// `concentric_lame_shells.rs`, and a dozen others): in a test, a violated
// invariant should abort loudly with its message rather than be threaded
// through a `Result` no caller can act on.
#![allow(clippy::expect_used)]

use sim_ml_chassis::Tensor;
use sim_soft::element::{Tet4, Tet10};
use sim_soft::mesh::HandBuiltTetMesh;
use sim_soft::solver::CpuNewtonSolver;
use sim_soft::{
    CpuTet4NHSolver, CpuTet10NHSolver, InitialGuess, MaterialField, Mesh, NullContact, Solver,
    Tet10Mesh,
};

// ---------------------------------------------------------------------------
// The fixture. Two free inputs (`EI_TARGET`, `MASS_PER_LENGTH`); the rest derives.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// The rig.
// ---------------------------------------------------------------------------

/// One solve's outcome, **including what happened when it failed**.
///
/// ⚠ A failed solve still carries a position (`x_partial`), so `ratio` and
/// `tip_m` are filled in either way. That is deliberate: a solve that reached a
/// sensible deflection and then missed an unreachably tight tolerance looks
/// identical to a real divergence unless the partial iterate is read. The two
/// are told apart by `r_norm` against `tol`, which is why both are carried.
struct Row {
    ratio: f64,
    tip_m: f64,
    free_dof: usize,
    converged: bool,
    /// Newton iterations the solve actually took. ★ Reported because a ratio
    /// that is right but took 300 iterations to reach says something about the
    /// fixture's conditioning that the ratio alone hides.
    iters: usize,
    /// `None` when converged; otherwise `(kind, r_norm, tol, iters)`.
    failure: Option<(&'static str, f64, f64, usize)>,
}

/// Solve one `(arm, nx, ny, nz)` cell of the sweep at stiffness `ei`.
fn solve(arm: Arm, nx: usize, ny: usize, nz: usize, ei: f64) -> Row {
    solve_at_tol(arm, nx, ny, nz, ei, TOL_REL)
}

/// [`solve`], with the tolerance exposed so a control can vary it.
fn solve_at_tol(arm: Arm, nx: usize, ny: usize, nz: usize, ei: f64, tol_rel: f64) -> Row {
    solve_full(arm, nx, ny, nz, ei, tol_rel, TARGET_RATIO)
}

/// [`solve_at_tol`], with the driven deflection ratio exposed too — the axis
/// that says whether a slow solve is nonlinearity or a bad tangent.
fn solve_full(
    arm: Arm,
    nx: usize,
    ny: usize,
    nz: usize,
    ei: f64,
    tol_rel: f64,
    target_ratio: f64,
) -> Row {
    let (mu, lambda) = lame_for(e_eff_for(ei));
    let field = MaterialField::uniform(mu, lambda);
    let load = load_for(ei) * (target_ratio / TARGET_RATIO);
    let tet4 = HandBuiltTetMesh::cantilever_bilayer_beam(nx, ny, nz, SPAN, WIDTH, DEPTH, &field);

    // `x_final` and the free-DOF count are all that leaves this match, so the
    // two element paths can be collapsed into one reporting tail.
    let (x_final_opt, loaded, rest_z, free_dof) = match arm {
        Arm::Tet10 => {
            let mesh = Tet10Mesh::from_tet4(&tet4);
            let n_dof = 3 * mesh.n_vertices();
            let (x_flat, rest_z, bc, cfg) = rig(&mesh, load, STATIC_DENSITY, false, tol_rel);
            let loaded = bc.loaded_vertices.clone();
            let free = n_dof - 3 * bc.pinned_vertices.len();
            let solver: CpuTet10NHSolver<Tet10Mesh> =
                CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc);
            let theta = Tensor::from_slice(&[load / loaded.len() as f64], &[1]);
            let out = outcome_of(
                solver.try_replay_step(
                    &Tensor::from_slice(&x_flat, &[n_dof]),
                    &Tensor::zeros(&[n_dof]),
                    &theta,
                    cfg.dt,
                ),
                cfg.tol,
                &x_flat,
            );
            (out, loaded, rest_z, free)
        }
        Arm::Tet4 | Arm::Tet4Fbar => {
            let n_dof = 3 * tet4.n_vertices();
            let (x_flat, rest_z, bc, cfg) =
                rig(&tet4, load, STATIC_DENSITY, arm == Arm::Tet4Fbar, tol_rel);
            let loaded = bc.loaded_vertices.clone();
            let free = n_dof - 3 * bc.pinned_vertices.len();
            let solver: CpuTet4NHSolver<HandBuiltTetMesh> =
                CpuNewtonSolver::new(Tet4, tet4, NullContact, cfg, bc);
            let theta = Tensor::from_slice(&[load / loaded.len() as f64], &[1]);
            let out = outcome_of(
                solver.try_replay_step(
                    &Tensor::from_slice(&x_flat, &[n_dof]),
                    &Tensor::zeros(&[n_dof]),
                    &theta,
                    cfg.dt,
                ),
                cfg.tol,
                &x_flat,
            );
            (out, loaded, rest_z, free)
        }
    };

    let (x_final, iters, failure) = x_final_opt;
    let tip = tip_of(&x_final, &loaded, &rest_z);
    Row {
        ratio: tip / (target_ratio * SPAN),
        tip_m: tip,
        free_dof,
        converged: failure.is_none(),
        iters,
        failure,
    }
}

// ---------------------------------------------------------------------------
// Gates.
// ---------------------------------------------------------------------------

/// The sizing preview — **free DOF per cell, no solves**.
///
/// Runs in debug too, because it is cheap and because it is the thing that has
/// to be read *before* the sweep's cost is committed.
#[test]
fn the_candidate_grid_sizes_are_reported_before_any_solve() {
    println!("\n=== stick fixture: free DOF by (element, nx, ny, nz) — NO SOLVES ===");
    println!(
        "  span {SPAN} m, section {WIDTH} x {DEPTH} m, slenderness {:.1}:1",
        SPAN / DEPTH
    );
    println!(
        "  EI = {EI_TARGET} N·m²  ->  E_eff = {:.2} GPa,  rho_eff = {:.0} kg/m³,  f1 = {:.1} Hz",
        e_eff_for(EI_TARGET) * 1e-9,
        rho_eff(),
        f1_analytic(EI_TARGET),
    );
    println!(
        "\n{:>4} {:>3} {:>3} {:>12} {:>12} {:>10}",
        "nx", "ny", "nz", "Tet4 DOF", "Tet10 DOF", "Tet10/Tet4"
    );

    let mut seen = 0usize;
    for (nx, ny, nz) in PREVIEW_GRIDS {
        let d4 = free_dof_of(Arm::Tet4, nx, ny, nz);
        let d10 = free_dof_of(Arm::Tet10, nx, ny, nz);
        assert!(
            d4 > 0 && d10 > d4,
            "nx={nx} nz={nz}: DOF counts are nonsense"
        );
        println!(
            "{nx:>4} {ny:>3} {nz:>3} {d4:>12} {d10:>12} {:>10.2}",
            d10 as f64 / d4 as f64
        );
        seen += 1;
    }
    // Counted against the array's OWN length, not against zero and not against a
    // hard-coded 13: `seen > 0` could not notice a `continue` added later, and a
    // literal count would fail spuriously the moment a grid is added.
    assert_eq!(
        seen,
        PREVIEW_GRIDS.len(),
        "the sizing table printed {seen} of {} designed grids — rows are being skipped",
        PREVIEW_GRIDS.len()
    );
    // ★ The superset relation is DOCUMENTED on `PREVIEW_GRIDS`, so enforce it:
    // a grid added to `GRIDS` and forgotten here would leave the sweep running
    // cells whose size was never reported, which is the one thing this preview
    // exists to prevent.
    for g in GRIDS {
        assert!(
            PREVIEW_GRIDS.contains(&g),
            "{g:?} is swept by GRIDS but not sized by PREVIEW_GRIDS, so the sweep runs a cell \
             whose DOF count is never reported"
        );
    }
}

/// Grids the sizing preview reports, a superset of [`GRIDS`] — it also sizes
/// the `ny = 2` cells the sweep uses only as a control.
const PREVIEW_GRIDS: [(usize, usize, usize); 13] = [
    (2, 1, 2),
    (4, 1, 2),
    (8, 1, 2),
    (16, 1, 2),
    (24, 1, 2),
    (32, 1, 2),
    (48, 1, 2),
    (8, 1, 4),
    (16, 1, 4),
    (24, 1, 4),
    (32, 1, 4),
    (16, 2, 4),
    (32, 2, 4),
];

/// The grids swept, coarsest first. `nz` is the **flex-direction** resolution
/// and the one bending accuracy should be most sensitive to.
const GRIDS: [(usize, usize, usize); 11] = [
    (2, 1, 2),
    (4, 1, 2),
    (8, 1, 2),
    (16, 1, 2),
    (24, 1, 2),
    (32, 1, 2),
    (48, 1, 2),
    (8, 1, 4),
    (16, 1, 4),
    (24, 1, 4),
    (32, 1, 4),
];

const ARMS: [Arm; 3] = [Arm::Tet4, Arm::Tet4Fbar, Arm::Tet10];

/// The accuracy bar a stick has to clear to "flex right", as a fraction of the
/// Euler–Bernoulli deflection.
///
/// ⚠ **Piloted, and it is a judgement, not a measurement.** A game's bar is
/// *visual*, not gradient-grade: `0.95` is a stick 5 % too stiff, which by the
/// `1/√r` relation is a `2.6 %` error in the whip frequency — well under what a
/// player could see. It sits below every Tet10 reading from `nx = 4` up
/// (`0.9501, 0.9780, 0.9902, …`) and far above every Tet4 reading in the swept
/// range (best `0.4001`), so it separates the two elements cleanly rather than
/// splitting either one's band.
const FLEX_BAR: f64 = 0.95;

/// Implied first-mode frequency error of a mesh reading `ratio` of analytic.
///
/// `f₁ ∝ √(EI_eff)`, and a mesh reading `r` is `1/r` too stiff, so `f₁` is
/// `1/√r` too high. ⚠ Arithmetic on the static result, **not** a measurement of
/// the dynamic mode — that is a separate rung.
fn implied_f1_error_pct(ratio: f64) -> f64 {
    (ratio.sqrt().recip() - 1.0) * 100.0
}

/// ★★★ **The sweep: fem/analytic against resolution, for three elements.**
///
/// Reports rather than gates the accuracy band — the band is set from this
/// table, not guessed ahead of it. What is asserted here is structural: every
/// cell must converge (a non-converged cell is not evidence), and the arms must
/// be measuring the same problem.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn stick_flex_accuracy_against_resolution() {
    println!("\n=== stick flex vs Euler-Bernoulli: fem/analytic by resolution and element ===");
    println!(
        "  span {SPAN} m, section {WIDTH} x {DEPTH} m (flex dir = z, depth), slenderness {:.1}:1",
        SPAN / DEPTH
    );
    println!(
        "  EI = {EI_TARGET} N·m2, E_eff = {:.2} GPa, nu = {NU}, load = {:.4} N -> delta_exact = {:.4} mm",
        e_eff_for(EI_TARGET) * 1e-9,
        load_for(EI_TARGET),
        TARGET_RATIO * SPAN * 1e3,
    );
    println!(
        "\n{:>4} {:>3} {:>3} {:>12} {:>9} {:>12} {:>10} {:>12}",
        "nx", "ny", "nz", "element", "free DOF", "delta (mm)", "fem/exact", "implied f1 %"
    );
    // ★ Newton iterations are printed alongside because a 300-iteration solve
    // and a 3-iteration solve can report the same ratio, and only one of them
    // is a fixture worth building a cost measurement on.

    let mut cells = 0usize;
    let mut nonconverged: Vec<String> = Vec::new();
    let mut best_linear = 0.0f64;
    // ★ Captured so the gates below assert on the numbers actually PRINTED,
    // rather than on three fresh solves of the same cells.
    let mut tet10_ratios: Vec<(usize, f64)> = Vec::new();

    for (nx, ny, nz) in GRIDS {
        for arm in ARMS {
            let row = solve(arm, nx, ny, nz, EI_TARGET);
            let dof = row.free_dof;
            if row.converged {
                println!(
                    "{nx:>4} {ny:>3} {nz:>3} {:>12} {dof:>9} {:>12.5} {:>10.4} {:>+12.1} {:>7}",
                    arm.label(),
                    row.tip_m * 1e3,
                    row.ratio,
                    implied_f1_error_pct(row.ratio),
                    row.iters,
                );
                // ★★ BOTH linear arms, **F-bar included**. F-bar IS the "cure"
                // the gate below names, and it reads HIGHER than plain Tet4 at
                // every grid (0.5178 vs 0.4001) — so excluding it would hide the
                // arm CLOSEST to the bar from the gate built to catch exactly
                // that. Reviewed in 2026-08-24's pass; the first version
                // accumulated `Arm::Tet4` only.
                if arm != Arm::Tet10 {
                    best_linear = best_linear.max(row.ratio);
                }
                if arm == Arm::Tet10 && ny == 1 && nz == 2 {
                    tet10_ratios.push((nx, row.ratio));
                }
            } else {
                let (kind, r_norm, tol, n) = row.failure.expect("a non-converged row must say why");
                println!(
                    "{nx:>4} {ny:>3} {nz:>3} {:>12} {dof:>9} {:>12.5} {:>10.4} {:>12}",
                    arm.label(),
                    row.tip_m * 1e3,
                    row.ratio,
                    format!("{kind}@{n}"),
                );
                println!(
                    "{:>46}   r_norm = {r_norm:.3e}  tol = {tol:.3e}  r/tol = {:.2e}",
                    "",
                    r_norm / tol
                );
                nonconverged.push(format!("{} nx={nx} ny={ny} nz={nz}", arm.label()));
            }
            cells += 1;
        }
    }

    cells += report_width_control();

    assert_eq!(
        cells,
        GRIDS.len() * ARMS.len() + ARMS.len(),
        "the table above must cover every cell of the designed matrix or it is a different \
         measurement than the one reported"
    );
    assert!(
        nonconverged.is_empty(),
        "a non-converged cell is not evidence; these did not converge: {nonconverged:?}"
    );
    assert!(
        best_linear > 0.0,
        "no linear-element row was measured, so there is nothing to compare the quadratic \
         element against"
    );

    gate_the_two_claims(&tet10_ratios, best_linear);
}

/// The claims [`stick_flex_accuracy_against_resolution`] makes, asserted against
/// the rows it actually printed.
fn gate_the_two_claims(tet10_ratios: &[(usize, f64)], best_linear: f64) {
    // Reads the sweep's own rows. ⚠ Panics rather than re-solving if a grid is
    // removed from `GRIDS`: a gate that quietly re-computes what it is meant to
    // check is no longer checking the reported measurement.
    let tet10_at = |nx: usize| {
        // ⚠ `assert!` then `expect`, rather than `expect` alone, ONLY so the
        // message can name `nx`. The first lint-clean rewrite of this used a
        // bare `.expect("this nx …")` and silently dropped which grid was
        // missing — losing information from a failure path to satisfy a lint is
        // the exact mistake this file was built around.
        let hit = tet10_ratios.iter().find(|(n, _)| *n == nx);
        assert!(
            hit.is_some(),
            "nx={nx} at ny=1,nz=2 is not among the sweep's captured rows, so this gate cannot \
             read the table it claims to gate. If the row exists but did not converge, the \
             non-convergence assert above should have fired first — check that it still runs \
             BEFORE this one"
        );
        hit.expect("asserted non-None on the line above").1
    };
    let coarse = tet10_at(8);
    assert!(
        coarse >= FLEX_BAR,
        "Tet10 at 720 free DOF read {coarse:.4} of analytic, below the {FLEX_BAR} bar \
         (measured 0.9780 on 2026-08-24). This file's headline — that a convincing stick is \
         SMALL — rests on this cell"
    );

    // Refinement must move TOWARD the closed form, which is the direction
    // h-refinement is obliged to move. ⚠ The predecessor of this rig in
    // `slender_bending_matches_analytic.rs` passed a gate while converging to
    // the wrong answer, because the quantity it was unknowingly measuring was
    // mesh-independent.
    let (fine, finer) = (tet10_at(16), tet10_at(48));
    assert!(
        coarse < fine && fine < finer && finer <= 1.02,
        "Tet10 must converge UP toward analytic under refinement; got \
         {coarse:.4} -> {fine:.4} -> {finer:.4}"
    );

    // ★★ Two-sided, and it fires on an IMPROVEMENT. `best_linear` is the best
    // reading from EITHER linear arm anywhere in the swept range — plain Tet4
    // (0.4001 at 864 free DOF) or Tet4+F-bar (0.5178, the higher of the two, and
    // therefore the one that would trip this first). If a future element or cure
    // lifts a linear arm over the bar, this file's central comparison — "the
    // quadratic element is not optional here" — has changed and must be
    // re-read, not silently kept.
    assert!(
        best_linear < FLEX_BAR,
        "a LINEAR arm (Tet4 or Tet4+F-bar) now reaches {best_linear:.4} of analytic, at or \
         above the {FLEX_BAR} bar. That is GOOD NEWS and it invalidates this file's conclusion: \
         re-read the element comparison and the cost table, which assume no linear arm reaches \
         the accuracy bar at any swept size"
    );
}

/// ★★ **The verdict does not depend on the stiffness calibration.**
///
/// [`EI_TARGET`] is one reading of a flex rating whose test span is not
/// standardised; another reading of the same rating gives `~880 N·m²`. That is
/// a `2.8×` spread, and if the discretisation verdict moved with it, this whole
/// file would be reporting a guess.
///
/// It does not move, and the reason is structural: `fem/analytic` is a ratio in
/// which `E` appears in the numerator *and* the denominator. Both the FEM
/// response and `δ = PL³/(3EI)` scale as `1/E`, and the load is re-derived per
/// stiffness to hold the deflection ratio fixed. What survives is the *element*
/// and the *mesh*. ⚠ It is only exactly `E`-independent because the response is
/// linear at `δ/L = 1e-3`; at slapshot amplitude it would not be, which is one
/// more reason that regime needs its own rung.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn the_discretisation_verdict_does_not_depend_on_stiffness() {
    const ALT_EI: f64 = 880.0;
    println!("\n=== stiffness-independence control: EI = {EI_TARGET} vs {ALT_EI} N·m2 ===");
    println!(
        "{:>12} {:>12} {:>12} {:>12}",
        "element", "ratio @310", "ratio @880", "rel. drift"
    );

    let mut worst_drift = 0.0f64;
    let mut compared = 0usize;
    for arm in ARMS {
        let a = solve(arm, 16, 1, 2, EI_TARGET);
        let b = solve(arm, 16, 1, 2, ALT_EI);
        assert!(
            a.converged && b.converged,
            "{}: both stiffnesses must converge for the comparison to mean anything",
            arm.label()
        );
        assert!(
            a.ratio.is_finite() && a.ratio > 0.0,
            "{}: baseline ratio {} is not a usable denominator",
            arm.label(),
            a.ratio
        );
        let drift = (b.ratio - a.ratio).abs() / a.ratio;
        println!(
            "{:>12} {:>12.5} {:>12.5} {:>12.2e}",
            arm.label(),
            a.ratio,
            b.ratio,
            drift
        );
        worst_drift = worst_drift.max(drift);
        compared += 1;
    }

    assert_eq!(
        compared,
        ARMS.len(),
        "every arm must be compared or the control covers less than it claims"
    );
    // Piloted below; a 2.8x stiffness change moving the ratio by more than this
    // would mean the sweep is reporting the calibration rather than the mesh.
    assert!(
        worst_drift < 1.0e-3,
        "the discretisation verdict moved with the stiffness calibration \
         (worst relative drift {worst_drift:.3e} across a 2.8x change in EI). \
         Either the response is no longer linear at delta/L = {TARGET_RATIO}, or the \
         load is not being re-derived per stiffness"
    );
}

/// ★★ **The verdict does not depend on the relaxed tolerance** — swept, and
/// with a positive control that proves the tolerance is connected at all.
///
/// [`TOL_REL`] was moved from `1e-6` to `1e-4` because the tighter value sat
/// below the f64 assembly floor for an `8 GPa` beam and stalled 17 of 30 cells
/// *at the right answer* ([`rig`] carries the evidence). A relaxation taken to
/// make a red gate green is worthless unless the answer is shown to be the same
/// answer.
///
/// # ⚠⚠ The first version of this control was a TAUTOLOGY
///
/// It compared `1e-4` against `1e-5` and asserted the ratios agreed. They agreed
/// at `0.00e0` — **because those two tolerances select the identical Newton
/// iterate**. Measured in review: iteration counts are `4 → 4`, `5 → 5`,
/// `163 → 163`. The control was comparing a computation against *itself*, so its
/// green was arithmetic identity rather than evidence. Worse, mutating the loose
/// arm by `1000×` (to `1e-1`) still did not move it — a control that survives
/// mutation of the very quantity it claims to test.
///
/// # What the sweep shows instead, which is stronger
///
/// Newton's residual falls in discrete jumps, so any tolerance inside one jump
/// selects the same iterate. Across `1e-1 … 1e-5` — **four orders** — every arm
/// returns a bit-identical ratio and iteration count. That is much better
/// evidence than the original single-decade comparison could give.
///
/// # ★ The positive control, without which "insensitive" is unfalsifiable
///
/// An inert knob and a **disconnected** knob look identical from inside a
/// negative control. So this also asserts the tolerance *can* change the answer:
/// at `tol_rel = 1e0` it collapses — Tet4 to `0.0275` in one iteration, and
/// **Tet10 to `0.0` in ZERO iterations**, the solver reporting success having
/// accepted the rest state. ⚠ Worth carrying beyond this test: `converged` from
/// this solver does not imply it did any work.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn the_verdict_does_not_depend_on_the_tolerance() {
    /// The working range, swept. Bounded below by the arithmetic floor (`1e-6`
    /// stalls) and above by [`TOL_DISCONNECT_PROBE`].
    const TOL_SPAN: [f64; 5] = [1.0e-1, 1.0e-2, 1.0e-3, 1.0e-4, 1.0e-5];
    /// A tolerance so slack the answer MUST move. Piloted: the smallest decade
    /// that changes any arm — everything from `1e-1` down is inert.
    const TOL_DISCONNECT_PROBE: f64 = 1.0e0;

    println!(
        "\n=== tolerance-independence: swept {:e}..{:e}, plus a disconnect probe ===",
        TOL_SPAN[0],
        TOL_SPAN[TOL_SPAN.len() - 1]
    );
    println!(
        "{:>12} {:>10} {:>16} {:>8} {:>12}",
        "element", "tol_rel", "ratio", "iters", "vs TOL_REL"
    );

    let mut compared = 0usize;
    let mut worst = 0.0f64;
    for arm in ARMS {
        let reference = solve_at_tol(arm, 16, 1, 2, EI_TARGET, TOL_REL);
        assert!(
            reference.converged && reference.ratio.is_finite() && reference.ratio > 0.0,
            "{}: the chosen tolerance must itself converge to a usable ratio; got {:?}",
            arm.label(),
            reference.failure
        );

        for tol in TOL_SPAN {
            let row = solve_at_tol(arm, 16, 1, 2, EI_TARGET, tol);
            assert!(
                row.converged,
                "{} at tol_rel={tol:e} must converge, or the swept range is wrong; got {:?}",
                arm.label(),
                row.failure
            );
            let drift = (row.ratio - reference.ratio).abs() / reference.ratio;
            println!(
                "{:>12} {tol:>10.0e} {:>16.8} {:>8} {:>12.2e}",
                arm.label(),
                row.ratio,
                row.iters,
                drift
            );
            worst = worst.max(drift);
            compared += 1;
        }

        // ★ POSITIVE control. Without it, "the ratio never moves" is equally
        // consistent with `tol_rel` never reaching the solver at all.
        let probe = solve_at_tol(arm, 16, 1, 2, EI_TARGET, TOL_DISCONNECT_PROBE);
        let moved = (probe.ratio - reference.ratio).abs() / reference.ratio;
        println!(
            "{:>12} {TOL_DISCONNECT_PROBE:>10.0e} {:>16.8} {:>8} {:>12.2e}  <- disconnect probe",
            arm.label(),
            probe.ratio,
            probe.iters,
            moved
        );
        assert!(
            moved > 0.5,
            "{}: at tol_rel={TOL_DISCONNECT_PROBE:e} the ratio moved only {moved:.3e}. A \
             tolerance that cannot change the answer even when made absurd is NOT WIRED to \
             the solver, and every 'insensitive' reading above is then meaningless",
            arm.label()
        );
    }

    assert_eq!(
        compared,
        ARMS.len() * TOL_SPAN.len(),
        "every arm must be swept across the whole span"
    );
    assert!(
        worst < 1.0e-4,
        "the reported ratios moved with the tolerance across the working range (worst \
         relative drift {worst:.3e}), so {TOL_REL:e} is not converged and this file's numbers \
         are tolerance artefacts"
    );
}

// ---------------------------------------------------------------------------
// Instrument B — what a stick costs per frame.
//
// The accuracy sweep above says a convincing stick is small. This half says
// what small costs, in the regime a game actually runs: dynamic, `dt = 1/60`,
// under a slapshot-scale load, with the settled `Inertial` predictor.
// ---------------------------------------------------------------------------

mod refbox;
mod stickrig;

use stickrig::{
    Arm, DEPTH, EI_TARGET, NU, RAMP_FRAMES, SLAPSHOT_DEFLECTION, SPAN, STATIC_DENSITY,
    TARGET_RATIO, TOL_REL, WIDTH, describe_failure, e_eff_for, f1_analytic, free_dof_of, lame_for,
    load_for, one_frame, outcome_of, percentile, rho_eff, rig, slapshot_load, tip_of,
};

/// Game timestep.
const FRAME_DT: f64 = 1.0 / 60.0;

/// Frames measured — the last stretch of the ramp, where the load is deepest.
///
/// ⚠ This is the **smoothly-driven** regime: a shaft being loaded through a
/// shot, which is what a stick does for most of a shot and what a game's moving
/// hand constraints impose every frame. It is NOT the impact regime (a puck
/// striking the blade), which the ramp pilot shows is by far the expensive one
/// and which this file does not measure.
const MEASURED_FRAMES: usize = 20;

/// What one cell of the cost matrix produced.
struct Cost {
    free_dof: usize,
    iters: Vec<usize>,
    ms: Vec<f64>,
    /// Total Newton iterations spent getting the load on — the loading phase's
    /// cost, kept separate because it is a different regime from the measured
    /// frames and averaging the two would hide both.
    ramp_iters: usize,
    /// Tip deflection (m) at the last completed frame. ★ The physics the cost
    /// was paid for: a cheaper run that arrives somewhere else is not a cheaper
    /// run, and only comparing this across tolerances can tell them apart.
    tip_m: f64,
    /// `Some(reason)` if the run ended before its last frame.
    ended_early: Option<String>,
}

impl Cost {
    /// Median of this run's sample. ★ Delegates to `stickrig::percentile` so
    /// this fixture and `stick_impact.rs` cannot drift on what a percentile
    /// means.
    ///
    /// ⚠ That does NOT extend to `predictor_spike.rs`, which keeps its own
    /// private `rank_index`/`percentile` and never calls this one. The *rule* is
    /// byte-identical by hand, deliberately (see `stickrig::percentile`), but
    /// nothing structural enforces it — and the copies already differ at the
    /// edges: that one returns `0` on an empty sample where this one panics, and
    /// it does not reject `q` outside `[0, 1]`.
    ///
    /// ⚠ That function used to take the UPPER median and argue it was the
    /// conservative choice for a fits-the-budget claim. Both halves are gone: it
    /// now uses the standard nearest-rank `⌈q·n⌉ − 1`, and it retracts the
    /// conservatism argument by name (the bias flips sign depending on whether
    /// you are claiming a figure fits a budget or busts one). Every consumer of
    /// `Cost` here is an `#[ignore]`d timing instrument, so no accuracy figure
    /// this file publishes moved.
    fn p50(values: &[f64]) -> f64 {
        percentile(values, 0.5)
    }

    fn ms_p50(&self) -> f64 {
        Self::p50(&self.ms)
    }

    fn iters_p50(&self) -> f64 {
        Self::p50(&self.iters.iter().map(|&i| i as f64).collect::<Vec<_>>())
    }

    /// ★ The cost metric the element A/B turns on. Dividing out the iteration
    /// count removes the confound that the two elements are at **different
    /// deflections under the same force** — a locked Tet4 barely moves, so it is
    /// in a more nearly linear regime and needs fewer iterations. That is a real
    /// difference and it is reported separately; it must not be allowed to
    /// masquerade as a per-iteration cost difference.
    ///
    /// ⚠ **Inert on the data as measured**, and the A/B is honest only because
    /// of that: every measured frame in the smooth ramp regime runs at exactly
    /// **one** Newton iteration, so this reduces to `ms_p50` and the ratio the
    /// A/B reports is a raw wall-time ratio at matched DOF. Were iteration
    /// counts ever to vary, note this is `median(ms) / median(iters)`, which is
    /// NOT `median(ms/iters)` — the two diverge once the counts spread.
    fn ms_per_iter(&self) -> f64 {
        self.ms_p50() / self.iters_p50()
    }
}

/// Build and run one cell of the cost matrix.
fn cost_of(arm: Arm, nx: usize, ny: usize, nz: usize) -> Cost {
    cost_of_at_tol(arm, nx, ny, nz, TOL_REL)
}

/// [`cost_of`], with the tolerance exposed so a control can vary it.
fn cost_of_at_tol(arm: Arm, nx: usize, ny: usize, nz: usize, tol_rel: f64) -> Cost {
    let (mu, lambda) = lame_for(e_eff_for(EI_TARGET));
    let field = MaterialField::uniform(mu, lambda);
    let load = slapshot_load(EI_TARGET);
    let tet4 = HandBuiltTetMesh::cantilever_bilayer_beam(nx, ny, nz, SPAN, WIDTH, DEPTH, &field);

    let mut iters = Vec::with_capacity(MEASURED_FRAMES);
    let mut ms = Vec::with_capacity(MEASURED_FRAMES);
    let mut ended_early: Option<String> = None;
    let mut ramp_iters = 0usize;
    let tip_m;
    let free_dof;

    match arm {
        Arm::Tet10 => {
            let mesh = Tet10Mesh::from_tet4(&tet4);
            let n_dof = 3 * mesh.n_vertices();
            let (x_flat, rest_z, bc, mut cfg) = rig(&mesh, load, rho_eff(), false, tol_rel);
            cfg.dt = FRAME_DT;
            cfg.initial_guess = InitialGuess::Inertial;
            free_dof = n_dof - 3 * bc.pinned_vertices.len();
            let loaded = bc.loaded_vertices.clone();
            let n_loaded = loaded.len();
            let solver: CpuTet10NHSolver<Tet10Mesh> =
                CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc);
            let (mut x, mut v) = (x_flat, vec![0.0; n_dof]);
            for k in 0..RAMP_FRAMES + MEASURED_FRAMES {
                // Ramp CONTINUOUSLY across warmup and measurement, so every
                // measured frame is doing real work. ⚠ Ramp-then-HOLD was tried
                // first and is degenerate: with the load constant the stick
                // settles, the `Inertial` guess already meets tol, and the
                // solver returns `iter_count = 0` — a frame time that measures
                // the predictor, not the solve. The measured frames sit at the
                // END of the ramp, i.e. the deepest, hardest part of the shot.
                let frac = (k + 1) as f64 / (RAMP_FRAMES + MEASURED_FRAMES) as f64;
                let theta = Tensor::from_slice(&[load * frac / n_loaded as f64], &[1]);
                match one_frame(&solver, &mut x, &mut v, &theta, FRAME_DT) {
                    Ok((it, t)) => {
                        if k >= RAMP_FRAMES {
                            iters.push(it);
                            ms.push(t);
                        } else {
                            ramp_iters += it;
                        }
                    }
                    Err(why) => {
                        ended_early = Some(format!("frame {k}: {why}"));
                        break;
                    }
                }
            }
            tip_m = tip_of(&x, &loaded, &rest_z);
        }
        Arm::Tet4 | Arm::Tet4Fbar => {
            let n_dof = 3 * tet4.n_vertices();
            let (x_flat, rest_z, bc, mut cfg) =
                rig(&tet4, load, rho_eff(), arm == Arm::Tet4Fbar, tol_rel);
            cfg.dt = FRAME_DT;
            cfg.initial_guess = InitialGuess::Inertial;
            free_dof = n_dof - 3 * bc.pinned_vertices.len();
            let loaded = bc.loaded_vertices.clone();
            let n_loaded = loaded.len();
            let solver: CpuTet4NHSolver<HandBuiltTetMesh> =
                CpuNewtonSolver::new(Tet4, tet4, NullContact, cfg, bc);
            let (mut x, mut v) = (x_flat, vec![0.0; n_dof]);
            for k in 0..RAMP_FRAMES + MEASURED_FRAMES {
                // Ramp CONTINUOUSLY across warmup and measurement, so every
                // measured frame is doing real work. ⚠ Ramp-then-HOLD was tried
                // first and is degenerate: with the load constant the stick
                // settles, the `Inertial` guess already meets tol, and the
                // solver returns `iter_count = 0` — a frame time that measures
                // the predictor, not the solve. The measured frames sit at the
                // END of the ramp, i.e. the deepest, hardest part of the shot.
                let frac = (k + 1) as f64 / (RAMP_FRAMES + MEASURED_FRAMES) as f64;
                let theta = Tensor::from_slice(&[load * frac / n_loaded as f64], &[1]);
                match one_frame(&solver, &mut x, &mut v, &theta, FRAME_DT) {
                    Ok((it, t)) => {
                        if k >= RAMP_FRAMES {
                            iters.push(it);
                            ms.push(t);
                        } else {
                            ramp_iters += it;
                        }
                    }
                    Err(why) => {
                        ended_early = Some(format!("frame {k}: {why}"));
                        break;
                    }
                }
            }
            tip_m = tip_of(&x, &loaded, &rest_z);
        }
    }

    Cost {
        free_dof,
        iters,
        ms,
        ramp_iters,
        tip_m,
        ended_early,
    }
}

/// ★★★ **What a convincing stick costs per frame.**
///
/// Read against §2a's anchors, which are the reason this fixture exists:
/// `540` free DOF at `0.64 ms`/Newton-iteration (its `dt = 1/60` rows — see the
/// module docs for why that is not its per-iteration table's `0.57`) fits a
/// game physics budget with
/// no reduction at all; `3 000` at `8.4 ms` busts it on a single iteration.
/// ⚠ Both of those are **Tet4** figures, and the accuracy sweep above says a
/// stick needs **Tet10** — which carries a denser stencil per DOF. So §2a's
/// anchors cannot simply be read off at the stick's DOF count; that is exactly
/// what the matched-DOF A/B below measures.
#[test]
#[ignore = "timing instrument — reference box only, run explicitly"]
fn stick_cost_per_frame() {
    refbox::require_quiet_box();
    println!("\n=== stick cost per frame: dt = 1/60, Inertial predictor, slapshot load ===");
    println!(
        "  load = {:.1} N (delta/L ~ {:.3}, the nonlinear regime), rho_eff = {:.0} kg/m3",
        slapshot_load(EI_TARGET),
        SLAPSHOT_DEFLECTION / SPAN,
        rho_eff(),
    );
    println!(
        "  load ramped CONTINUOUSLY over {} frames (piloted: a step load does not converge \
         at all);\n  the last {MEASURED_FRAMES} are measured, at the deepest part of the ramp\n",
        RAMP_FRAMES + MEASURED_FRAMES
    );
    println!(
        "{:>12} {:>4} {:>9} {:>11} {:>12} {:>12} {:>14} {:>11}",
        "element",
        "nx",
        "free DOF",
        "iters p50",
        "ms/frame p50",
        "ms/iter",
        "ms/frame range",
        "ramp iters"
    );

    let mut measured = 0usize;
    let mut verdict: Vec<(usize, f64)> = Vec::new();
    for (arm, nx) in [
        (Arm::Tet10, 4),
        (Arm::Tet10, 8),
        (Arm::Tet10, 16),
        (Arm::Tet4, 20),
        (Arm::Tet4, 40),
        (Arm::Tet4, 80),
    ] {
        let c = cost_of(arm, nx, 1, 2);
        assert_eq!(
            c.ms.len(),
            MEASURED_FRAMES,
            "{} nx={nx}: only {} of {MEASURED_FRAMES} frames completed — a run that gave up \
             early is not a cost measurement. Reason: {}",
            arm.label(),
            c.ms.len(),
            c.ended_early.as_deref().unwrap_or("(none recorded)")
        );
        assert!(
            c.iters_p50() > 0.0,
            "{} nx={nx}: median Newton iterations is ZERO — this cell measures the predictor \
             accepting its own guess, not a solve",
            arm.label()
        );
        let lo = c.ms.iter().copied().fold(f64::INFINITY, f64::min);
        let hi = c.ms.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        println!(
            "{:>12} {nx:>4} {:>9} {:>11.1} {:>12.3} {:>12.3} {:>14} {:>11}",
            arm.label(),
            c.free_dof,
            c.iters_p50(),
            c.ms_p50(),
            c.ms_per_iter(),
            format!("{lo:.2}-{hi:.2}"),
            c.ramp_iters,
        );
        if arm == Arm::Tet10 {
            verdict.push((c.free_dof, c.ms_p50()));
        }
        measured += 1;
    }
    assert_eq!(
        measured, 6,
        "the table above must cover the designed matrix"
    );

    // ---- The punchline, stated against budgetS rather than one budget. ----
    println!("\n  Tet10 stick against a frame budget:");
    println!(
        "{:>9} {:>12} {:>12} {:>12} {:>12}",
        "free DOF", "ms/frame", "x 16.7 ms", "x 6 ms", "x 4 ms"
    );
    for (dof, ms) in &verdict {
        println!(
            "{dof:>9} {ms:>12.2} {:>12.3} {:>12.3} {:>12.3}",
            ms / 16.7,
            ms / 6.0,
            ms / 4.0
        );
    }
    println!(
        "\n  WARNING: 16.7 ms is the WHOLE 60 Hz frame -- render, AI, broadphase, audio and\n  \
         gameplay come out of it too. '4-6 ms' is an ASSUMED physics allocation, not a\n  \
         stated requirement; the real budget is an open question for the target's owner.\n  \
         Quote ms/frame as a RANGE: repeat runs of this table move it by ~5%."
    );
    assert!(
        !verdict.is_empty(),
        "no Tet10 row reached the verdict table, so the budget comparison is vacuous"
    );
}

/// ★★ **What the quadratic element costs per DOF** — matched free DOF, one
/// process, interleaved, `p50` over frames.
///
/// This is the durable half of the cost story, and it is built as a **ratio in
/// a same-process A/B** rather than as an absolute time, because absolutes drift
/// across sessions and machine state while a ratio between two arms that share
/// that state largely does not.
///
/// The pairs carry **identical free DOF** by construction — `Tet10` at `nx` and
/// `Tet4` at `5·nx` both land on `18·nx` free DOF on this section — so the ratio
/// isolates the element's stencil density from its DOF count.
#[test]
#[ignore = "timing instrument — reference box only, run explicitly"]
fn a_tet10_dof_costs_more_than_a_tet4_dof() {
    refbox::require_quiet_box();
    println!("\n=== matched-DOF element A/B: what a Tet10 DOF costs vs a Tet4 DOF ===");
    println!(
        "{:>9} {:>16} {:>16} {:>12} {:>12}",
        "free DOF", "Tet10 ms/iter", "Tet4 ms/iter", "Tet10/Tet4", "iters 10 vs 4"
    );

    let mut ratios: Vec<f64> = Vec::new();
    for (nx10, nx4) in [(4usize, 20usize), (8, 40), (16, 80)] {
        let a = cost_of(Arm::Tet10, nx10, 1, 2);
        let b = cost_of(Arm::Tet4, nx4, 1, 2);
        assert_eq!(
            a.free_dof, b.free_dof,
            "the pair (Tet10 nx={nx10}, Tet4 nx={nx4}) is NOT matched on free DOF \
             ({} vs {}), so its ratio would confound element with size",
            a.free_dof, b.free_dof
        );
        assert!(
            a.ms.len() == MEASURED_FRAMES && b.ms.len() == MEASURED_FRAMES,
            "both arms must complete every frame for the ratio to mean anything"
        );
        // A zero-iteration frame means the predictor's guess already met tol —
        // the frame measured the predictor, not the solve — and it also makes
        // `ms_per_iter` infinite. Both arms must actually be solving.
        for (label, c) in [("Tet10", &a), ("Tet4", &b)] {
            assert!(
                c.iters_p50() > 0.0,
                "{label}: median Newton iterations is ZERO, so these frames measure the \
                 predictor accepting its own guess rather than a solve, and the cost ratio \
                 would be vacuous"
            );
            assert!(
                c.ms_per_iter().is_finite() && c.ms_per_iter() > 0.0,
                "{label}: ms/iteration is {} — not a usable measurement",
                c.ms_per_iter()
            );
        }
        let ratio = a.ms_per_iter() / b.ms_per_iter();
        println!(
            "{:>9} {:>16.4} {:>16.4} {:>12.2} {:>12}",
            a.free_dof,
            a.ms_per_iter(),
            b.ms_per_iter(),
            ratio,
            format!("{:.0} vs {:.0}", a.iters_p50(), b.iters_p50()),
        );
        ratios.push(ratio);
    }

    assert_eq!(ratios.len(), 3, "every matched pair must be measured");
    assert!(
        ratios.iter().all(|r| r.is_finite() && *r > 0.0),
        "a non-finite or non-positive cost ratio is not a measurement: {ratios:?}"
    );
}

/// **Where the dynamic rig loses convergence** — a four-axis discriminator.
///
/// The first cost run hit the iteration cap with `r_norm` stuck at `1.36e1`, and
/// a load sweep then showed the stall is **not** a magnitude problem in the
/// ordinary sense: `r_norm` scales *linearly* with the load, stalling at a fixed
/// ~25 % of it at every magnitude and both mesh sizes, and even 2 % of a
/// slapshot needs 279 iterations. A fixed residual *fraction*, independent of
/// magnitude, is a structural signature, not a nonlinearity.
///
/// The static half of this file solves the same meshes without trouble, so what
/// changed is `ρ = 0 → 454`, `dt = 1 → 1/60`, and the predictor. This varies
/// those one at a time instead of guessing which one it is.
///
/// ⚠ The `density = 0, dt = 1` row is the **static rig at the slapshot load** —
/// the cell that separates "the dynamics broke it" from "the load is simply
/// beyond this fixture's geometric-nonlinearity range".
#[test]
#[ignore = "diagnostic — run explicitly"]
fn where_the_dynamic_rig_loses_convergence() {
    let load = slapshot_load(EI_TARGET);
    println!("\n=== dynamic-rig discriminator (Tet10/Tet4, nx as marked, load = {load:.1} N) ===");
    println!(
        "{:>12} {:>4} {:>10} {:>9} {:>14} {:>8} {:>40}",
        "element", "nx", "density", "dt", "predictor", "iters", "outcome"
    );

    let mut converged_any = false;
    let mut probed = 0usize;
    for (arm, nx) in [(Arm::Tet10, 8usize), (Arm::Tet4, 40usize)] {
        for (rho_label, rho) in [("0", 0.0), ("rho_eff", rho_eff())] {
            for (dt_label, dt) in [("1.0", 1.0), ("1/60", FRAME_DT), ("1/600", FRAME_DT / 10.0)] {
                for (g_label, guess) in [
                    ("PreviousState", InitialGuess::PreviousState),
                    ("Inertial", InitialGuess::Inertial),
                ] {
                    let out = probe_cell(arm, nx, load, rho, dt, guess);
                    match out {
                        Ok(it) => {
                            println!(
                                "{:>12} {nx:>4} {rho_label:>10} {dt_label:>9} {g_label:>14} \
                                 {it:>8} {:>40}",
                                arm.label(),
                                "ok"
                            );
                            converged_any = true;
                        }
                        Err(why) => println!(
                            "{:>12} {nx:>4} {rho_label:>10} {dt_label:>9} {g_label:>14} \
                             {:>8} {why:>40}",
                            arm.label(),
                            "-"
                        ),
                    }
                    probed += 1;
                }
            }
        }
    }
    // ⚠ Round two's commit claimed BOTH `assert!(probed > 0)` counters were
    // replaced. Only one was — this is the other, found by inventorying all 41
    // assertions rather than by re-reading the diff.
    assert_eq!(
        probed,
        2 * 2 * 3 * 2,
        "the discriminator printed {probed} of 24 designed cells — rows are being skipped"
    );
    assert!(
        converged_any,
        "NO cell of the discriminator converged, so it discriminates nothing — the rig \
         itself is wrong, not one of the four axes"
    );
}

/// One cell of [`where_the_dynamic_rig_loses_convergence`]: a single step from
/// rest under `load`, at the given mass, timestep and predictor.
fn probe_cell(
    arm: Arm,
    nx: usize,
    load: f64,
    density: f64,
    dt: f64,
    guess: InitialGuess,
) -> Result<usize, String> {
    let (mu, lambda) = lame_for(e_eff_for(EI_TARGET));
    let field = MaterialField::uniform(mu, lambda);
    let tet4 = HandBuiltTetMesh::cantilever_bilayer_beam(nx, 1, 2, SPAN, WIDTH, DEPTH, &field);
    match arm {
        Arm::Tet10 => {
            let mesh = Tet10Mesh::from_tet4(&tet4);
            let n_dof = 3 * mesh.n_vertices();
            let (x_flat, _, bc, mut cfg) = rig(&mesh, load, density, false, TOL_REL);
            cfg.dt = dt;
            cfg.initial_guess = guess;
            let n_loaded = bc.loaded_vertices.len();
            let solver: CpuTet10NHSolver<Tet10Mesh> =
                CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc);
            let theta = Tensor::from_slice(&[load / n_loaded as f64], &[1]);
            step_once(&solver, &x_flat, n_dof, &theta, dt)
        }
        Arm::Tet4 | Arm::Tet4Fbar => {
            let n_dof = 3 * tet4.n_vertices();
            let (x_flat, _, bc, mut cfg) = rig(&tet4, load, density, arm == Arm::Tet4Fbar, TOL_REL);
            cfg.dt = dt;
            cfg.initial_guess = guess;
            let n_loaded = bc.loaded_vertices.len();
            let solver: CpuTet4NHSolver<HandBuiltTetMesh> =
                CpuNewtonSolver::new(Tet4, tet4, NullContact, cfg, bc);
            let theta = Tensor::from_slice(&[load / n_loaded as f64], &[1]);
            step_once(&solver, &x_flat, n_dof, &theta, dt)
        }
    }
}

/// A single `try_replay_step` from rest, reporting iteration count or reason.
fn step_once<S: Solver>(
    solver: &S,
    x_flat: &[f64],
    n_dof: usize,
    theta: &Tensor<f64>,
    dt: f64,
) -> Result<usize, String> {
    solver
        .try_replay_step(
            &Tensor::from_slice(x_flat, &[n_dof]),
            &Tensor::zeros(&[n_dof]),
            theta,
            dt,
        )
        .map(|s| s.iter_count)
        .map_err(|e| describe_failure(&e))
}

/// ★★★ **Is Tet10's 262-iteration solve nonlinearity, or a bad tangent?**
///
/// The accuracy sweep reports the right answer and an alarming way of reaching
/// it: driven to `δ/L = 1e-3` — near-linear — **Tet4 takes 2–29 Newton
/// iterations and Tet10 takes 111–367**, worsening as the mesh coarsens. That
/// matters beyond tidiness, because cost per frame is `ms/iteration ×
/// iterations/step`: a `100×` iteration penalty would erase the DOF advantage
/// the accuracy sweep just established.
///
/// ## ⚠ The probe that was here first could not have discriminated anything
///
/// It swept `EI` across six decades and read "iterations flat in stiffness ⇒
/// the grind is the element". The reading was **262 iterations at every single
/// stiffness, exactly** — and that flatness is guaranteed by construction, not
/// evidence: the load is re-derived per `EI` to hold `δ/L` fixed, which makes
/// the problem *exactly scale-invariant*, so Newton retraces an identical
/// trajectory. The probe could only ever have printed one answer. It is kept
/// only as the amplitude axis below, which is the axis that can actually fail.
///
/// ## The discriminator that works
///
/// Drive the amplitude toward zero. As `δ/L → 0` the problem becomes **linear**,
/// and Newton with a tangent that is the true derivative of the residual solves
/// a linear problem in **one step**. So:
///
/// - iterations fall toward `1–2` as amplitude falls ⇒ the tangent is right and
///   the grind is genuine geometric nonlinearity;
/// - iterations stay near `262` at `δ/L = 1e-6` ⇒ **Tet10's forward tangent is
///   not the derivative of its residual**, which is a solver defect, outside
///   this file's scope, and needs its own arc.
#[test]
#[ignore = "diagnostic — run explicitly"]
fn whether_the_iteration_grind_is_nonlinearity_or_the_tangent() {
    println!("\n=== Newton iterations vs driven amplitude (EI held at {EI_TARGET}) ===");
    println!(
        "  As delta/L -> 0 the problem becomes LINEAR. An exact tangent solves a linear\n           problem in ONE Newton step, so iterations must collapse. If they do not, the\n           tangent is not the derivative of the residual.\n"
    );
    println!(
        "{:>12} {:>12} {:>10} {:>8} {:>14}",
        "delta/L", "element", "fem/exact", "iters", "outcome"
    );

    // `(amplitude, iterations, ratio)` — the ratio is what tells a fast stall
    // apart from a fast stall AT THE WRONG ANSWER.
    let mut tet10: Vec<(f64, usize, f64)> = Vec::new();
    let mut tet4: Vec<(f64, usize, f64)> = Vec::new();
    for ratio in [1.0e-8, 1.0e-6, 1.0e-4, 1.0e-3, 1.0e-2] {
        for (arm, nx) in [(Arm::Tet10, 8usize), (Arm::Tet4, 40usize)] {
            let row = solve_full(arm, nx, 1, 2, EI_TARGET, TOL_REL, ratio);
            // ⚠ A row that did NOT converge must not print like one that did.
            // The `δ/L = 1e-2` Tet10 cell hits the iteration cap and reports
            // `0.2285` — a partial iterate, NOT a converged wrong answer, and
            // the distinction is this whole file's founding lesson.
            println!(
                "{ratio:>12.0e} {:>12} {:>10.4} {:>8} {:>14}",
                arm.label(),
                row.ratio,
                row.iters,
                row.failure.map_or("converged", |(kind, ..)| kind),
            );
            if arm == Arm::Tet10 {
                tet10.push((ratio, row.iters, row.ratio));
            } else {
                tet4.push((ratio, row.iters, row.ratio));
            }
        }
    }

    let smallest = |v: &[(f64, usize, f64)]| v.first().map_or(usize::MAX, |&(_, i, _)| i);
    let largest = |v: &[(f64, usize, f64)]| v.last().map_or(0, |&(_, i, _)| i);
    assert!(
        tet10.len() == 5 && tet4.len() == 5,
        "every amplitude must be measured for both elements"
    );
    println!(
        "\n  Tet10: {} iters at the SMALLEST amplitude, {} at the largest",
        smallest(&tet10),
        largest(&tet10)
    );
    println!(
        "  Tet4 : {} iters at the SMALLEST amplitude, {} at the largest",
        smallest(&tet4),
        largest(&tet4)
    );
    // ⚠⚠ **The smallest-amplitude rows ARMIJO-STALL, they do not converge**, and
    // that was invisible until the `outcome` column was added in review. `tol` is
    // `1e-4·load/√n` and the load shrinks with the amplitude, so at `δ/L = 1e-8`
    // the target is back under the arithmetic floor — the same trap the static
    // sweep hit, arriving from the other direction.
    //
    // ★ It does not overturn the reading, but it changes what the reading IS. A
    // low iteration count at a stall means "reached the floor fast", which is
    // only evidence of a good tangent if the answer it reached is the RIGHT one.
    // So the verdict requires BOTH: few iterations, AND the same ratio the
    // converged rows report. A bad tangent grinding slowly would still be far
    // from the answer at iteration 9.
    let converged_ratio = tet10
        .iter()
        .find(|(r, _, _)| (*r - 1.0e-3).abs() < 1e-12)
        .map(|&(_, _, ratio)| ratio)
        .expect("the delta/L = 1e-3 row is the converged reference and must be present");
    // ⚠ Found BY MINIMUM AMPLITUDE, not by position. `smallest`/`largest` below
    // still read `.first()`/`.last()`, which silently mislabels if the amplitude
    // array is ever reordered; this check must not inherit that.
    let &(_, _, linear_ratio) = tet10
        .iter()
        .min_by(|a, b| a.0.partial_cmp(&b.0).expect("amplitudes are never NaN"))
        .expect("the amplitude sweep is non-empty");
    let ratio_agrees = (linear_ratio - converged_ratio).abs() / converged_ratio < 1.0e-3;
    println!(
        "  linear-limit ratio {linear_ratio:.4} vs converged {converged_ratio:.4} — {}",
        if ratio_agrees { "AGREE" } else { "DISAGREE" }
    );
    println!(
        "  => {}",
        // Piloted: the linear-limit readings are Tet10 7-9 and Tet4 5-10, against
        // 262 and 16 at delta/L = 1e-3. A tangent that was not the derivative
        // would hold its iteration count as the problem linearised; both
        // elements drop by 1-2 orders instead. `15` sits above the measured
        // linear-limit noise and far below any non-collapse.
        if smallest(&tet10) <= 15 && ratio_agrees {
            "collapses toward linear AT THE RIGHT ANSWER: the tangent is FINE, the grind is \
             real NONLINEARITY"
        } else if ratio_agrees {
            "does NOT collapse on a linear problem: the TANGENT is the suspect"
        } else {
            "the linear-limit rows do not agree with the converged ratio — this probe is \
             measuring something other than the tangent and cannot be read"
        }
    );
    assert!(
        ratio_agrees,
        "the linear-limit row reports {linear_ratio:.4} against the converged {converged_ratio:.4}. \
         Those rows ARMIJO-STALL, and a stall at the WRONG answer makes the iteration count \
         above meaningless as evidence about the tangent"
    );
}

/// **Does load continuation clear the convergence wall?**
///
/// A step load is unphysical — a slapshot loads the shaft over the `50–100 ms`
/// the blade is in contact, which is `3–6` frames at 60 Hz — and it is also the
/// hardest thing to ask Newton for, since each frame starts from a state far
/// from its own solution. Ramping is therefore both the right physics and
/// standard continuation.
///
/// The question this settles is which of two very different statements is true:
///
/// - **"Step loads are unphysical."** The ramp converges, the cost measurement
///   is recoverable, and the wall was an artefact of the loading protocol.
/// - **"The solver cannot reach stick amplitudes."** The ramp does not converge
///   either, and then no amount of model-order reduction helps — reduction cuts
///   `ms/iteration`, never the iteration count, so the wall would sit squarely
///   on the irreducible side of the frame budget.
#[test]
#[ignore = "diagnostic — run explicitly"]
fn whether_load_continuation_clears_the_wall() {
    let full = slapshot_load(EI_TARGET);
    println!("\n=== load continuation: ramp {full:.1} N over N frames, dt = 1/60 ===");
    println!(
        "{:>12} {:>4} {:>7} {:>9} {:>13} {:>10} {:>34}",
        "element", "nx", "ramp", "reached", "iters (last)", "iters sum", "outcome"
    );

    let mut any_ok = false;
    let mut probed = 0usize;
    for (arm, nx) in [(Arm::Tet10, 8usize), (Arm::Tet4, 40usize)] {
        for ramp in [6usize, 20, 60, 240] {
            let (reached, last_iters, total, why) = ramp_run(arm, nx, full, ramp);
            println!(
                "{:>12} {nx:>4} {ramp:>7} {:>9} {last_iters:>13} {total:>10} {:>34}",
                arm.label(),
                format!("{reached}/{ramp}"),
                why.as_deref().unwrap_or("ok"),
            );
            if why.is_none() {
                any_ok = true;
            }
            probed += 1;
        }
    }
    assert_eq!(
        probed,
        2 * 4,
        "the ramp table printed {probed} of 8 designed cells — rows are being skipped"
    );
    println!(
        "\n  => {}",
        if any_ok {
            "CONTINUATION CLEARS IT: the wall was the step-load protocol"
        } else {
            "CONTINUATION DOES NOT CLEAR IT: the wall is the solver at stick amplitudes, \
             and reduction cannot move it"
        }
    );
    // ★★ ASSERTED, not just printed. Found by mutation: collapsing every ramp to
    // a single frame — i.e. back to the step load that started this whole
    // investigation — left this test PASSING, because its conclusion lived only
    // in a `println!`. A verdict nobody gates is a verdict nobody is watching.
    //
    // ⚠ Deliberately one-sided in the direction that matters: `RAMP_FRAMES = 60`
    // is piloted off this table, so if continuation ever stops clearing the wall
    // the cost instrument's whole regime is invalid and must be re-piloted.
    assert!(
        any_ok,
        "NO ramp length cleared the load, so continuation no longer rescues the step-load \
         wall — `RAMP_FRAMES` is piloted off this table and the cost instrument's regime \
         must be re-derived before any timing figure is quoted"
    );
}

/// Ramp `full` linearly over `ramp` frames. Returns
/// `(frames reached, iterations on the last completed frame, total iterations,
/// failure reason)`.
fn ramp_run(arm: Arm, nx: usize, full: f64, ramp: usize) -> (usize, usize, usize, Option<String>) {
    let (mu, lambda) = lame_for(e_eff_for(EI_TARGET));
    let field = MaterialField::uniform(mu, lambda);
    let tet4 = HandBuiltTetMesh::cantilever_bilayer_beam(nx, 1, 2, SPAN, WIDTH, DEPTH, &field);

    macro_rules! run {
        ($mesh:expr, $elem:expr, $solver_ty:ty) => {{
            let mesh = $mesh;
            let n_dof = 3 * mesh.n_vertices();
            let (x_flat, _, bc, mut cfg) = rig(&mesh, full, rho_eff(), false, TOL_REL);
            cfg.dt = FRAME_DT;
            cfg.initial_guess = InitialGuess::Inertial;
            let n_loaded = bc.loaded_vertices.len();
            let solver: $solver_ty = CpuNewtonSolver::new($elem, mesh, NullContact, cfg, bc);
            let (mut x, mut v) = (x_flat, vec![0.0; n_dof]);
            let (mut done, mut last, mut total, mut why) = (0usize, 0usize, 0usize, None);
            for k in 0..ramp {
                let frac = (k + 1) as f64 / ramp as f64;
                let theta = Tensor::from_slice(&[full * frac / n_loaded as f64], &[1]);
                match one_frame(&solver, &mut x, &mut v, &theta, FRAME_DT) {
                    Ok((it, _)) => {
                        done = k + 1;
                        last = it;
                        total += it;
                    }
                    Err(e) => {
                        why = Some(format!("frame {k}: {e}"));
                        break;
                    }
                }
            }
            (done, last, total, why)
        }};
    }

    match arm {
        Arm::Tet10 => run!(
            Tet10Mesh::from_tet4(&tet4),
            Tet10,
            CpuTet10NHSolver<Tet10Mesh>
        ),
        Arm::Tet4 | Arm::Tet4Fbar => run!(tet4, Tet4, CpuTet4NHSolver<HandBuiltTetMesh>),
    }
}

/// ★★ **The cost figures are not bought with a loose tolerance.**
///
/// The cost table reports **one Newton iteration per frame**, which is exactly
/// the reading that would appear if the tolerance were too slack to make the
/// solver work — and the ramp-then-hold protocol already produced a genuinely
/// vacuous version of this (`iter_count = 0`, a frame time that measured the
/// predictor). One iteration is not zero, but "not vacuous" is not "converged".
///
/// So this tightens by `10×` and checks **both** halves of the claim:
///
/// - the **trajectory** must be the same — a cheaper run that arrives somewhere
///   else is not a cheaper run;
/// - the cost must not collapse under the tighter tolerance, which is what would
///   show the reported figure to be an artefact of the slack one.
///
/// ⚠ Two-sided deliberately: this fails if the tip deflection moves, and it
/// fails if the tighter arm cannot complete its frames.
#[test]
#[ignore = "timing instrument — reference box only, run explicitly"]
fn the_cost_figures_are_not_a_loose_tolerance_artefact() {
    const TIGHTER: f64 = TOL_REL / 10.0;
    refbox::require_quiet_box();
    println!("\n=== cost tolerance control: {TOL_REL:e} vs {TIGHTER:e} (relative) ===");
    println!(
        "{:>12} {:>9} {:>11} {:>11} {:>13} {:>13} {:>12}",
        "element",
        "free DOF",
        "iters @1e-4",
        "iters @1e-5",
        "tip @1e-4 mm",
        "tip @1e-5 mm",
        "tip drift"
    );

    let mut compared = 0usize;
    let mut worst_tip_drift = 0.0f64;
    let mut iters_rose = false;
    for (arm, nx) in [(Arm::Tet10, 8usize), (Arm::Tet4, 40usize)] {
        let loose = cost_of_at_tol(arm, nx, 1, 2, TOL_REL);
        let tight = cost_of_at_tol(arm, nx, 1, 2, TIGHTER);
        for (label, c) in [("loose", &loose), ("tight", &tight)] {
            assert_eq!(
                c.ms.len(),
                MEASURED_FRAMES,
                "{} nx={nx} ({label}): only {} of {MEASURED_FRAMES} frames completed. Reason: {}",
                arm.label(),
                c.ms.len(),
                c.ended_early.as_deref().unwrap_or("(none recorded)")
            );
            assert!(
                c.tip_m.is_finite() && c.tip_m > 0.0,
                "{} ({label}): tip deflection {} is not a usable trajectory readout",
                arm.label(),
                c.tip_m
            );
        }
        let drift = (tight.tip_m - loose.tip_m).abs() / loose.tip_m;
        println!(
            "{:>12} {:>9} {:>11.1} {:>11.1} {:>13.4} {:>13.4} {:>12.2e}",
            arm.label(),
            loose.free_dof,
            loose.iters_p50(),
            tight.iters_p50(),
            loose.tip_m * 1e3,
            tight.tip_m * 1e3,
            drift,
        );
        worst_tip_drift = worst_tip_drift.max(drift);
        iters_rose |= tight.iters_p50() > loose.iters_p50();
        compared += 1;
    }

    assert_eq!(compared, 2, "both elements must be compared");
    // ★ The wiring check, and the reason this control is not the tautology its
    // static sibling was: at least one arm must actually SPEND more iterations
    // under the tighter tolerance. Measured Tet10 `1 -> 2` and Tet4 `1 -> 1`, so
    // it is asserted over the pair rather than per-arm. Without this, "the
    // trajectory did not move" is equally consistent with the tolerance never
    // reaching the solver.
    assert!(
        iters_rose,
        "no arm spent more Newton iterations under a 10x tighter tolerance, so this control \
         cannot distinguish a converged trajectory from a tolerance that is not wired to the \
         solver at all"
    );
    assert!(
        worst_tip_drift < 1.0e-3,
        "the trajectory moved with the tolerance (worst relative tip drift \
         {worst_tip_drift:.3e}), so {TOL_REL:e} is not converged and the cost table is \
         timing a different simulation than the one it claims"
    );
}

/// The `ny = 2` control rows: doubling the resolution across the beam's WIDTH
/// should barely move a bending answer, because bending is resolved along the
/// span and through the depth. Reported as part of the sweep, and lifted out of
/// it only to keep that function readable.
fn report_width_control() -> usize {
    println!("\n  ny = 2 control (width resolution should barely move bending):");
    let mut rows = 0usize;
    for arm in ARMS {
        let row = solve(arm, 16, 2, 4, EI_TARGET);
        println!(
            "{:>4} {:>3} {:>3} {:>12} {:>9} {:>12.5} {:>10.4} {:>+12.1}",
            16,
            2,
            4,
            arm.label(),
            row.free_dof,
            row.tip_m * 1e3,
            row.ratio,
            implied_f1_error_pct(row.ratio),
        );
        rows += 1;
    }
    rows
}
