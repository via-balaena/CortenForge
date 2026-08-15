//! ★★★ **Solid tets do NOT lock in slender bending. The "ceiling" was this
//! file's own rig.**
//!
//! A Tet10 cantilever at 20:1 recovers **0.97 of the analytic small-deflection
//! answer** at the coarsest grid measured, and climbs to **0.995** under
//! refinement. Tet4 locks, as a linear element should; the quadratic element
//! does not.
//!
//! # What this file used to claim, and why that claim is gone
//!
//! It was named `slender_bending_ceiling.rs`, and it asserted the opposite:
//! that a 20:1 Tet10 cantilever lands near **a quarter** of analytic, that this
//! is mesh-converged both ways, and that element order does not rescue it — so
//! the answer had to be a different element **family**, hexahedra. That
//! conclusion was recorded as a measured ceiling capping "a hockey stick, a leaf
//! spring, an exo strut, a rib".
//!
//! **Every leg of it inverts, because the rig was not static.**
//!
//! ```text
//!                          OLD RIG (dt = 1, ρ = 1030)   STATIC     analytic
//!   Tet10 20:1, nx = 8              0.2493              0.9696       1.0
//!   Tet10 20:1, nx = 64             0.2512              0.9954       1.0
//!   Tet10/Tet4 gain, 5:1 → 20:1   1.75 → 1.70x       1.89 → 3.54x
//! ```
//!
//! The old gain was flat, which is exactly the signature the file said would
//! rule out p-refinement and select a new element family. Made static, the gain
//! **grows** with slenderness — the other branch of that same discriminator.
//!
//! # The mechanism
//!
//! `STATIC_DT` was `1.0`, under a comment reading "quasi-static: one step, no
//! dynamics". Backward Euler with `v₀ = 0` contributes `M(x − x₀)/dt²` to the
//! residual, so at `dt = 1` the added stiffness is **`M` itself**, on every
//! node, and the result was then compared against `δ = PL³/(3EI)` — a *static*
//! closed form.
//!
//! It is fatal here specifically because a slender cantilever is extremely
//! compliant. At 20:1 the beam is `1.5 × 0.075 × 0.075 m` — **8.69 kg** at
//! ρ = 1030 — against a tip structural stiffness of only
//! `P/δ = 9.49e-4 / 1.5e-3 = 0.633 N/m`. The parasitic term is several times
//! the structure's own stiffness, which lands the ratio near a quarter.
//!
//! ⚠ **This is why the bug hid.** The error scales as `M/(dt²·k)`, so it bites
//! only a very compliant structure — and a 20:1 cantilever driven in bending is
//! about as compliant as this engine is ever asked to be.
//!
//! The repo's own convention already says how to avoid it: `DiscParams::static_dt`
//! is `1.0e3`, documented "large, so the inertial term `M/dt²` is negligible",
//! and it is threaded straight to `cfg.dt` (`sim/L1/coupling/src/bonded.rs`), so
//! **`cf-fsu-model`'s `k_disc` ladder is not affected** — checked, not assumed.
//! `tet10_face_contact` also uses `1.0e3`. This file used `1.0`, which is `1e6×`
//! more inertia than the convention it sat beside.
//!
//! Here the rig asks for **`density = 0`** instead: the measurement is a static
//! benchmark against a static closed form, so inertia is not part of it, and
//! saying that outright is stronger than choosing a `dt` large enough to hide it.
//!
//! ## ✅ The blast radius was audited; this rig was the only casualty
//!
//! ⚠ This section once said "**two** other rigs still use `STATIC_DT = 1.0`" — a
//! number from a truncated `grep`. **Sixteen files define it**: fifteen
//! `sim-soft` tests (this one included) and one research tool. Thirteen were
//! A/B'd at `density = 0` and **all pass** — named, so the claim is auditable:
//! `tet10_bending_locking`, `bonded_bilayer_beam`, `fbar_locking` (beams);
//! `hertz_sphere_plane`, `bonded_layer_indentation`, `tet10_indentation_demand1`
//! (Hertz); `concentric_lame_shells` (Lamé); `contact_stability`,
//! `penalty_compressive_block`, `non_interpenetration`, `deformed_validity`,
//! `contact_grad_hook`, `material_grad_hook`. So the contamination was isolated
//! to the one rig comparing a *very compliant* structure against a *static*
//! closed form; elsewhere `M/(dt²·k)` either vanishes or cancels in a ratio.
//! (`contact_drop_rest` is *not* in that set — it runs at `dt = 1e-3`, and fails
//! under the probe exactly as a gravity-driven transient with no mass should,
//! which is what proved the probe reached these rigs at all.)
//!
//! ⚠ "Pass" bounds the shift rather than showing it is nil — the bands are ±5 %.
//! Measured where that mattered: `fbar_locking` moves ~1 %, and
//! `tet10_bending_locking`'s deflections **+0.3 % to +2.4 %**, so its Tet4
//! "~46 % of analytic" reads 46.9 % (holds) and its Tet10 "95 %" reads 97.4 %
//! (mildly conservative). ★ Every shift makes the engine look *more* accurate, as
//! this mechanism must **for rigs solving from rest** (`v_prev = 0`), where
//! `M(x − x₀)/dt²` is a positive-definite pull toward `x₀`. ⚠ Not general: with
//! `v₀ ≠ 0` it pulls toward `x₀ + dt·v₀` and can act either way.
//!
//! ⛔ **Two are not audited.** `tet10_lame_decision` (the exiled 7.16 GB / ~60 min
//! probe), and `cf-sim-research`'s `insertion_sim.rs` — a research tool whose own
//! comment reads "`dt = 1.0` collapses inertia for a quasi-static solve", the very
//! belief corrected here. Whether it matters there is unmeasured.
//!
//! ★ Method worth reusing: the A/B zeroed the **one** place density becomes mass
//! (`construct.rs`), not sixteen test files — and the probe was checked to *bite*
//! first, gate 4 flipping 0.2493 → 0.9696 under it. A probe that silently did
//! nothing would have reported all sixteen clear.
//!
//! # How the correction was established
//!
//! Not by re-reading the code. A standalone linear-elastic FEM — different
//! language, different formulation, no Newton, no `NeoHookean`, no mass term by
//! construction — was written against the same grid, the same CFK 6-tet split,
//! the same `L/μ/ν`, the same load rule and the same clamp. With inertia removed
//! from this rig the two agree **to four decimals on every arm**:
//!
//! ```text
//!   nx = 16      Tet4 (indep / here)      Tet10 (indep / here)     gain
//!     5:1          0.5319 / 0.5319          1.0078 / 1.0078        1.89x
//!    10:1          0.4396 / 0.4396          0.9903 / 0.9903        2.25x
//!    20:1          0.2783 / 0.2783          0.9863 / 0.9863        3.54x
//! ```
//!
//! # What is asserted here
//!
//! 1. [`a_slender_tet10_cantilever_recovers_the_analytic_deflection`] — the
//!    magnitude, against the closed form.
//! 2. [`refinement_converges_toward_the_analytic_answer`] — it converges *up*,
//!    the direction h-refinement must move.
//! 3. [`element_order_rescues_slender_bending_and_the_gain_grows`] — the
//!    discriminator, now reading the other way.
//! 4. ★★ [`the_inertia_term_is_what_manufactured_the_old_ceiling`] — the
//!    negative control. It **reproduces the artifact on demand**: restore
//!    `ρ = 1030` at `dt = 1` and the same mesh collapses to ~0.25. Without it,
//!    (1)–(3) are just numbers that happen to be near 1.0 and the mechanism
//!    behind a wrong architectural conclusion stays undocumented.
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
//! ▶ The VR frame-budget question this file originally asked — "can a flexing
//! Tet10 stick fit 11.1 ms at 90 Hz" — was parked as "blocked behind the
//! element". **It is unblocked**: the element was never the problem.

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

/// Quasi-static timestep. ⚠ Only meaningful together with [`STATIC_DENSITY`] —
/// see the module docs. A `dt` alone does not make a step static.
const STATIC_DT: f64 = 1.0;

/// ★★ **Zero, and that is the whole correction.** Backward Euler adds
/// `M(x − x₀)/dt²`; this measurement is a static benchmark against a static
/// closed form, so there is no inertia in it. At `ρ = 1030` and `dt = 1` this
/// rig read a 20:1 Tet10 cantilever as 0.2493 of analytic and that number was
/// recorded as a property of tetrahedra.
const STATIC_DENSITY: f64 = 0.0;

/// The density the artifact needs, used *only* by the negative control.
const ARTIFACT_DENSITY: f64 = 1030.0;

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
/// ⚠ The tolerance is relative to the load, and the load here is tiny (~9e-4 N):
/// a `1e-9` relative tolerance would ask for a ~1e-14 residual, which is machine
/// epsilon and unreachable. A tolerance is only meaningful once its ABSOLUTE
/// scale has been checked against the problem.
fn rig<M: Mesh>(
    mesh: &M,
    load: f64,
    density: f64,
) -> (Vec<f64>, Vec<f64>, BoundaryConditions, SolverConfig) {
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
    cfg.density = density;
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
fn tet10_ratio_at(aspect: f64, nx: usize, nz: usize, density: f64) -> (f64, f64, bool) {
    let h = L / aspect;
    let load = load_for(h);
    let lambda = 2.0 * MU * NU / (1.0 - 2.0 * NU);
    let field = MaterialField::uniform(MU, lambda);
    let tet4 = HandBuiltTetMesh::cantilever_bilayer_beam(nx, 2, nz, L, h, h, &field);
    let mesh = Tet10Mesh::from_tet4(&tet4);
    let n_dof = 3 * mesh.n_vertices();
    let (x_flat, rest_z, bc, cfg) = rig(&mesh, load, density);
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

fn tet10_ratio(aspect: f64, nx: usize, nz: usize) -> (f64, f64, bool) {
    tet10_ratio_at(aspect, nx, nz, STATIC_DENSITY)
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
    let (x_flat, rest_z, bc, cfg) = rig(&mesh, load, STATIC_DENSITY);
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

/// ★★★ **A slender Tet10 cantilever recovers the analytic deflection.**
///
/// The claim this file exists to make. `tet10_bending_locking` validates a
/// *ratio* — Tet10 vs Tet4 — at 5:1; this is the absolute check against a closed
/// form, at slenderness, which is what was missing when the ceiling was
/// "measured".
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn a_slender_tet10_cantilever_recovers_the_analytic_deflection() {
    println!("\n=== Tet10 vs analytic small-deflection cantilever (STATIC) ===");
    println!("  δ_analytic = P L³ / (3 E I), driven at δ/L ≈ 1e-3 so Euler–Bernoulli is exact.\n");
    println!(
        "{:>8} {:>6} {:>14} {:>12}",
        "aspect", "nz", "δ_fem (mm)", "fem/exact"
    );

    let mut worst = f64::INFINITY;
    let mut slender_rows = 0usize;
    for aspect in [5.0, 10.0, 20.0] {
        for nz in [2, 4] {
            let (ratio, tip, ok) = tet10_ratio(aspect, 8, nz);
            println!("{aspect:>8.0} {nz:>6} {:>14.5} {ratio:>12.4}", tip * 1e3);
            assert!(
                ok,
                "aspect {aspect}:1 nz={nz} must converge to be evidence at all"
            );
            if aspect >= 20.0 {
                slender_rows += 1;
                worst = worst.min(ratio);
            }
        }
    }

    assert!(
        slender_rows > 0,
        "no slender row was measured, so the figures above are vacuous"
    );
    // Below 1.0 because this is the COARSEST grid here (nx = 8); gate 2 walks it
    // up to 0.995. A reading materially under this band means the inertia term —
    // or something like it — is back.
    assert!(
        (0.95..1.02).contains(&worst),
        "expected a 20:1 Tet10 cantilever to recover the analytic deflection \
         (measured 0.9696 at nx=8 on 2026-08-14); got {worst:.4}. A collapse toward \
         0.25 is the signature of the backward-Euler inertia term returning — check \
         `STATIC_DENSITY` before concluding anything about the element"
    );
}

/// ★★ **It converges UP toward the analytic answer**, which is the direction
/// h-refinement must move.
///
/// ⚠ The predecessor of this gate asserted the opposite — that refinement moved
/// the answer < 10 % and stayed under 0.40 — and it *passed*, because the
/// inertia term it was unknowingly measuring is mesh-independent. A gate can be
/// green, converged and precise about the wrong quantity.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn refinement_converges_toward_the_analytic_answer() {
    println!("\n=== Mesh convergence at 20:1, both directions (STATIC) ===");
    println!(
        "{:>5} {:>5} {:>14} {:>12}",
        "nx", "nz", "elem aspect", "fem/exact"
    );

    let h = L / 20.0;
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

    println!("\n  nx 8→64 climbs {coarse:.4} → {fine:.4}; nz 2→4 at nx=64 reads {thick:.4}.");

    assert!(
        fine > coarse,
        "refining along the beam must move the answer UP toward analytic; \
         got {coarse:.4} → {fine:.4}"
    );
    assert!(
        fine > 0.99,
        "a refined 20:1 Tet10 shaft must reach the analytic answer; got {fine:.4}"
    );
    assert!(
        (0.95..1.02).contains(&thick),
        "through-thickness refinement must stay on the analytic answer; got {thick:.4}"
    );
}

/// ★★ **Order rescues slender bending, and the gain GROWS with slenderness** —
/// the discriminator, now reading the other way.
///
/// The two branches were stated before either was measured:
///
/// - **gain grows with slenderness** ⇒ the error is dominated by element
///   **order**, and p-refinement is the path.
/// - **gain flat** ⇒ order does not rescue it, and the answer is a different
///   element **family** (hexahedra), a far larger addition.
///
/// Under the old rig the gain read flat (1.75 → 1.70×) and selected hexahedra.
/// Made static it reads **1.89 → 2.25 → 3.54×** and selects the first branch —
/// which the Tet10 column settles outright, since it is already *at* analytic.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn element_order_rescues_slender_bending_and_the_gain_grows() {
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
        assert!(ok4 && ok10, "aspect {aspect}:1 must converge on both arms");
        let gain = r10 / r4;
        println!("{aspect:>8.0} {r4:>12.4} {r10:>12.4} {gain:>11.2}x");
        if i == 0 {
            stubby_gain = gain;
        }
        slender_gain = gain;
    }

    // Directional: the discriminator is whether the gain GROWS or is flat, not
    // where it lands. The 1.10 is a noise deadband rather than a fitted
    // threshold — 1.89x → 3.54x clears it by a wide margin, and the old rig's
    // 1.75x → 1.70x fails it in the other direction.
    assert!(
        slender_gain > stubby_gain * 1.10,
        "the Tet10/Tet4 gain must GROW with slenderness for order to be the fix; \
         got {stubby_gain:.2}x at 5:1 and {slender_gain:.2}x at 20:1"
    );
}

/// ★★ **The negative control: reproduce the artifact on demand.**
///
/// Gates 1–3 assert numbers near 1.0. On their own that is indistinguishable
/// from a rig that cannot produce anything else, and it leaves no executable
/// record of the mechanism that produced a wrong architectural conclusion. So
/// this restores the one knob — `ρ = 1030` at `dt = 1` — and requires the old
/// number back.
///
/// `M/dt²` at `dt = 1` is `M`. The 20:1 beam is `1.5 × 0.075 × 0.075 m`, i.e.
/// **8.69 kg**, against a tip structural stiffness of `P/δ = 0.633 N/m`. The
/// parasitic term is several times the structure's own stiffness, and the ratio
/// lands near a quarter.
///
/// ⚠ If this stops reproducing ~0.25, the explanation for the old ceiling is
/// wrong and this file's entire narrative needs re-deriving — that is a louder
/// failure than a drifted threshold, and it is meant to be.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn the_inertia_term_is_what_manufactured_the_old_ceiling() {
    println!("\n=== Negative control: restore the inertia term, get the artifact back ===");

    let (statik, _, ok_s) = tet10_ratio_at(20.0, 8, 2, STATIC_DENSITY);
    let (inertial, _, ok_i) = tet10_ratio_at(20.0, 8, 2, ARTIFACT_DENSITY);
    assert!(ok_s && ok_i, "both arms must converge to be evidence");

    println!("  ρ = 0     (static)   fem/exact {statik:>8.4}");
    println!("  ρ = 1030  (dt = 1)   fem/exact {inertial:>8.4}   <- the recorded 'ceiling'");
    println!(
        "\n  One knob, {:.1}x apart. The element is identical in both rows.",
        statik / inertial
    );

    assert!(
        (0.20..0.30).contains(&inertial),
        "restoring ρ = 1030 at dt = 1 must reproduce the old ~0.25 reading \
         (it was 0.2493); got {inertial:.4}. If this moved, the diagnosis of the \
         old ceiling no longer holds"
    );
    assert!(
        statik > 3.0 * inertial,
        "the static and inertial readings must be far apart — that separation IS \
         the finding; got {statik:.4} vs {inertial:.4}"
    );
}
