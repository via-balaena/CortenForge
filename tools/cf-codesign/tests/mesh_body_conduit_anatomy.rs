//! Env-gated gate: routing a [`ConduitTarget`] around a **real L4 vertebra** — the
//! real-anatomy rung of the geometry axis.
//!
//! Where `mesh_body_conduit.rs` routes around a meshed *analytic* sphere (exact field
//! known, quantitative anchors), this routes around an actual BodyParts3D lumbar
//! vertebra: thin transverse/spinous processes, the vertebral foramen, concavities.
//! The mesh is licensed (CC BY-SA) and never committed, so every test here is
//! `#[ignore]`d and reads the STL from `$CF_L4_STL` — see
//! `tests/assets/bodyparts3d/PROVENANCE.md` for the source + fetch. Run with:
//!
//! ```sh
//! CF_L4_STL=/tmp/FMA13075.stl cargo test -p cf-codesign --release \
//!   --test mesh_body_conduit_anatomy -- --ignored --nocapture
//! ```
//!
//! The closed-form `r*` prediction does not transfer to real anatomy (neither the
//! binding-sample count nor the mean binding clearance is known), so the falsifiable
//! anchors are behavioural: the field is **FD-stable**, the conduit **clears** the
//! vertebra from a start that pierces it, its radius **grows with the reward**, and the
//! recovered design is **stable across grid resolution**.
//!
//! **Coordinates.** The mesh is used in its native millimetre frame (positioned in the
//! whole-atlas frame, `body_center` near `z ≈ 970 mm`). No rescale to SI is done — not
//! because the objective is scale-free (it is not: the target's finite-difference step
//! and the optimiser's `grad_tol` are absolute — see
//! [`OptConfig::grad_tol`](cf_codesign::OptConfig)), but because *every* constant here —
//! `MARGIN`, `CELL`, `PAD`, `OVERHANG`, the target's FD step, and the optimiser's
//! `grad_tol`/`lr` — is consistently in millimetres. The SI-metre requirement is
//! specific to the soft FEM solver, which this pure-geometry gate does not use.
//!
//! **On the field.** The grid-cached signed distance is FD-stable on this geometry
//! (test `anatomy_conduit_field_is_fd_stable`) but not perfectly *metric* near thin
//! features (`‖∇φ‖` wanders from 1 where the trilinear interpolant rounds a sharp
//! concavity). That only perturbs the optimiser's descent, not the clearance *values*
//! the other gates assert, so those anchors read φ, never `∇φ`.

use std::sync::Arc;

use cf_codesign::{CoDesignProblem, ConduitTarget, mesh_body, optimize};
use cf_design::{IndexedMesh, Sdf};
use cf_fsu_geometry::{body_center, load_from_env, oracle};
use nalgebra::Point3;

const MARGIN: f64 = 2.0; // mm — clearance demanded beyond the tube radius
const W_C: f64 = 10.0; // clearance penalty weight
const W_R: f64 = 1.0; // radius reward
const N_SAMPLES: usize = 60;
const CELL: f64 = 1.5; // signed-distance grid spacing (mm)
const OVERHANG: f64 = 15.0; // route extends this far beyond the vertebra AABB along x (mm)
const PAD: f64 = 40.0; // grid padding beyond the vertebra AABB (mm). > OVERHANG, so the
// route endpoints always sit inside the lattice for any L4 (independent of proportions),
// and deep enough to hold the +z detour + tube even at the largest reward tested.
// Off-ridge +z nudges (mm). This objective is multi-modal in the start: from a route
// that *pierces* the bone the cheapest move is to shrink the tube (smaller radius →
// smaller clearance demand) and slip out thin, whereas from a route that already
// *clears* the reward drives the radius up to the fattest-that-fits. So two seeds:
const PIERCE_LIFT: f64 = 8.0; // small — the seed still pierces a lumbar body (recovery-
// from-infeasible + a penalty-active point for the FD-stability probe).
const CLEAR_LIFT: f64 = 30.0; // past the vertebra's half-height — the seed already clears,
// so the reward, not the penalty, sets the radius (reward-response + resolution).
const R0: f64 = 3.0; // seed tube radius (mm)

/// Load + weld-repair the real L4 (native mm), locate its body centre (the deepest
/// interior point), and its x-extent. Returns `(mesh, body_center, (x_min, x_max))`.
fn load_l4() -> (IndexedMesh, Point3<f64>, (f64, f64)) {
    let l4 = load_from_env("CF_L4_STL").expect("$CF_L4_STL must point at the L4 STL");
    let o = oracle(&l4).expect("L4 oracle");
    let (bc, _depth) = body_center(&l4, &o);
    let (mut x_min, mut x_max) = (f64::INFINITY, f64::NEG_INFINITY);
    for v in &l4.vertices {
        x_min = x_min.min(v.x);
        x_max = x_max.max(v.x);
    }
    (l4, bc, (x_min, x_max))
}

/// A conduit routed along x, spanning the vertebra plus [`OVERHANG`] beyond each face,
/// through `bc` — so the straight line pierces the bone and a detour is forced. Grid
/// built at `cell`, reward `w_r`. Endpoints derive from the mesh's own x-extent
/// (`x_range`), so they clear the vertebra and stay inside the padded grid for any L4.
fn conduit(body: Arc<dyn Sdf>, x_range: (f64, f64), bc: Point3<f64>, w_r: f64) -> ConduitTarget {
    let start = Point3::new(x_range.0 - OVERHANG, bc.y, bc.z);
    let end = Point3::new(x_range.1 + OVERHANG, bc.y, bc.z);
    ConduitTarget::new(body, start, end, 2, MARGIN, W_C, w_r, N_SAMPLES)
}

/// The straight route at radius [`R0`], nudged in +z off the symmetric ridge by `lift`
/// mm on both interior control points. [`PIERCE_LIFT`] keeps the seed inside the bone;
/// [`CLEAR_LIFT`] lifts it clear (see the seed-constant note on why the start matters).
fn seed(t: &ConduitTarget, lift: f64) -> Vec<f64> {
    let mut x = t.x0(R0);
    x[2] += lift; // interior CP 0, z
    x[5] += lift; // interior CP 1, z
    x
}

/// (1) CLEARANCE INVARIANT, from a piercing start. The seed's route is inside the bone
/// — `min_clearance < 0`, asserted at runtime, not assumed — so recovery must climb it
/// out. The recovered conduit then clears the vertebra: the centreline is strictly
/// outside (`min_clearance > 0`) and the whole tube (radius `r` about it) clears
/// (`min_clearance ≥ r`). The pierce→clear pair is the negative control: a no-op
/// optimiser returns the still-piercing seed, whose `min_clearance < 0` fails the
/// `> 0` assertion. (From a piercing start the cheapest escape is to slip out thin, so
/// the recovered radius is small here — the fattest-that-fits regime is exercised from
/// a clearing start in the reward-response and resolution gates below.)
#[test]
#[ignore = "needs $CF_L4_STL (BodyParts3D FMA13075, CC BY-SA, not committed)"]
fn anatomy_conduit_clears_the_vertebra() {
    let (l4, bc, xr) = load_l4();
    let body = mesh_body(l4, CELL, PAD).expect("grid-cached L4 body");
    let t = conduit(body, xr, bc, W_R);

    let x0 = seed(&t, PIERCE_LIFT);
    let seed_clr = t.min_clearance(&x0);
    assert!(
        seed_clr < 0.0,
        "seed route must pierce the vertebra so clearing is real work (min_clearance {seed_clr:.2} mm)"
    );

    let res = optimize(&t, &x0, &t.recommended_config());
    let r = t.radius(&res.params);
    let clr = t.min_clearance(&res.params);
    let req = t.req_clearance(&res.params);
    println!(
        "[clears] seed_clr={seed_clr:.2}  ->  r={r:.3} mm  min_clearance={clr:.3} mm  req={req:.3} mm  iters={}  stop={:?}",
        res.iters, res.stop_reason
    );

    // The route climbed from inside the bone to outside — a no-op (returning the
    // piercing seed) fails this, since the seed's clearance is negative.
    assert!(
        clr > 0.0,
        "recovered centreline enters the vertebra: {clr:.3} mm"
    );
    // ...and the whole tube clears, not just the centreline.
    assert!(
        clr >= r,
        "the tube (radius {r:.3}) does not clear the bone: min_clearance {clr:.3} mm < r"
    );
}

/// (2) FD-STABILITY on real anatomy — the committed measurement behind the claim that
/// the grid field is smooth at the finite-difference scale. At a piercing start (penalty
/// active, so the mesh field drives the gradient), the target's internal FD (eps 1e-6)
/// agrees with an independent FD at eps 1e-4 — a 100× larger step. Agreement across that
/// range is the falsifiable form of "the trilinear field is smooth at the FD scale"; a
/// facet/cell kink would blow the two apart. Mirrors the sphere gate's
/// `conduit_over_mesh_body_gradient_is_fd_stable`, now on a busy real vertebra.
#[test]
#[ignore = "needs $CF_L4_STL (BodyParts3D FMA13075, CC BY-SA, not committed)"]
fn anatomy_conduit_field_is_fd_stable() {
    let (l4, bc, xr) = load_l4();
    let body = mesh_body(l4, CELL, PAD).expect("grid-cached L4 body");
    let t = conduit(body, xr, bc, W_R);

    let x = seed(&t, PIERCE_LIFT);
    let (_loss, grad) = t.evaluate(&x); // internal central FD at eps 1e-6
    let eps = 1e-4; // 100× the internal step
    let mut worst = 0.0_f64;
    for i in 0..grad.len() {
        let mut xp = x.clone();
        xp[i] += eps;
        let jp = t.objective(&xp);
        xp[i] -= 2.0 * eps;
        let jm = t.objective(&xp);
        let fd = (jp - jm) / (2.0 * eps);
        worst = worst.max((grad[i] - fd).abs() / grad[i].abs().max(1.0));
    }
    println!("[fd-stable] worst relative disagreement (eps 1e-6 vs 1e-4) = {worst:.3e}");
    // Measured ~6e-8 on real L4; a kink-dominated field would sit ~1e-2. The bar sits
    // decisively between, with ample headroom over the measurement.
    assert!(
        worst < 1.0e-5,
        "FD gradient not stable across a 100× step on real anatomy: worst rel {worst:.3e}"
    );
}

/// (3) MONOTONE IN THE REWARD: a larger radius reward buys a fatter conduit. Around a
/// compact body in free space there is no clearance ceiling (the route can always
/// detour further), so `r*` grows with `w_r` — a behavioural check that holds on any
/// geometry, replacing the corridor scene's closed-form `r*`. One grid, two rewards.
#[test]
#[ignore = "needs $CF_L4_STL (BodyParts3D FMA13075, CC BY-SA, not committed)"]
fn anatomy_conduit_radius_grows_with_reward() {
    let (l4, bc, xr) = load_l4();
    let body = mesh_body(l4, CELL, PAD).expect("grid-cached L4 body");

    let recover = |w_r: f64| {
        let t = conduit(Arc::clone(&body), xr, bc, w_r);
        let res = optimize(&t, &seed(&t, CLEAR_LIFT), &t.recommended_config());
        t.radius(&res.params)
    };
    let (r_lo, r_hi) = (recover(0.5), recover(3.0));
    println!("[reward] r(w_r=0.5)={r_lo:.3} mm  r(w_r=3.0)={r_hi:.3} mm");

    assert!(
        r_hi > r_lo + 1.0,
        "a larger reward must fatten the conduit: r(3.0)={r_hi:.3} not clearly > r(0.5)={r_lo:.3}"
    );
}

/// (4) GRID-RESOLUTION STABILITY: the recovered design must not be an artefact of the
/// lattice spacing. Halving the cells (finer grid) should move the recovered radius and
/// detour only within a small multiple of the cell — the representation-convergence
/// anchor that stands in for the sphere gate's mesh-vs-analytic cross-check (there is no
/// analytic ground truth for a real vertebra). Both runs share the seed and config and
/// stop at the same limit cycle, so this measures resolution stability *up to* that
/// shared dynamics — a genuine but weaker signal than a converged cross-check.
#[test]
#[ignore = "needs $CF_L4_STL (BodyParts3D FMA13075, CC BY-SA, not committed)"]
fn anatomy_conduit_stable_across_grid_resolution() {
    let (l4, bc, xr) = load_l4();

    let recover = |cell: f64| {
        let body = mesh_body(l4.clone(), cell, PAD).expect("grid-cached L4 body");
        let t = conduit(body, xr, bc, W_R);
        let res = optimize(&t, &seed(&t, CLEAR_LIFT), &t.recommended_config());
        // Mean +z bulge of the interior control points *relative to* the body centre
        // (the CP z-coords sit in the ~970 mm atlas frame; subtract it for the detour).
        let detour = (res.params[2] + res.params[5]) / 2.0 - bc.z;
        (t.radius(&res.params), detour)
    };
    let (r_fine, d_fine) = recover(1.0);
    let (r_coarse, d_coarse) = recover(2.0);
    println!(
        "[resolution] cell 1.0: r={r_fine:.3} detour={d_fine:.3}  |  cell 2.0: r={r_coarse:.3} detour={d_coarse:.3}"
    );

    // Measured drift halving the cell is sub-cell (~0.13 mm radius / ~0.07 mm detour).
    // The bars sit a few multiples above that yet well below one cell (1–2 mm), so a
    // resolution-dependent artefact fails while the limit-cycle jitter passes.
    assert!(
        (r_fine - r_coarse).abs() < 0.6,
        "recovered radius drifts with grid resolution: {r_fine:.3} (fine) vs {r_coarse:.3} (coarse) mm"
    );
    assert!(
        (d_fine - d_coarse).abs() < 0.5,
        "recovered detour drifts with grid resolution: {d_fine:.3} (fine) vs {d_coarse:.3} (coarse) mm"
    );
}
