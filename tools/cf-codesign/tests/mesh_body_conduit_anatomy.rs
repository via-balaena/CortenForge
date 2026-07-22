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
//! anchors are behavioural: the conduit **clears** the vertebra, its radius **grows
//! with the reward**, and the recovered design is **stable across grid resolution**.
//! The mesh works in its native millimetre frame (the objective is scale-agnostic
//! geometry — see the PROVENANCE note).
//!
//! **On the field.** The grid-cached signed distance is FD-stable on this geometry
//! (measured ~1e-9 relative across a 100× step change) but not perfectly *metric* near
//! thin features (`‖∇φ‖` wanders from 1 where the trilinear interpolant rounds a sharp
//! concavity). That only perturbs the optimiser's descent, not the clearance *values*
//! these gates assert, and recovery still converges — so the anchors read φ, never
//! `∇φ`.

use std::sync::Arc;

use cf_codesign::{ConduitTarget, mesh_body, optimize};
use cf_design::Sdf;
use cf_fsu_geometry::{body_center, load_from_env, oracle};
use nalgebra::Point3;

const MARGIN: f64 = 2.0; // mm — clearance demanded beyond the tube radius
const W_C: f64 = 10.0; // clearance penalty weight
const W_R: f64 = 1.0; // radius reward
const N_SAMPLES: usize = 60;
const CELL: f64 = 1.5; // signed-distance grid spacing (mm)
const PAD: f64 = 40.0; // grid padding beyond the vertebra AABB (mm) — covers the route
const HALF_SPAN: f64 = 70.0; // route endpoints at body_center ± this along x (mm)

/// Load + weld-repair the real L4 (native mm) and locate its body centre — the deepest
/// interior point, which a straight route through it is guaranteed to pierce.
fn load_l4() -> (cf_design::IndexedMesh, Point3<f64>) {
    let l4 = load_from_env("CF_L4_STL").expect("$CF_L4_STL must point at the L4 STL");
    let o = oracle(&l4).expect("L4 oracle");
    let (bc, _depth) = body_center(&l4, &o);
    (l4, bc)
}

/// A conduit routed through `bc` along x (the vertebra's long axis), so the straight
/// line pierces the bone and a detour is forced. Grid built at `cell`, reward `w_r`.
fn conduit(body: Arc<dyn Sdf>, bc: Point3<f64>, w_r: f64) -> ConduitTarget {
    let start = Point3::new(bc.x - HALF_SPAN, bc.y, bc.z);
    let end = Point3::new(bc.x + HALF_SPAN, bc.y, bc.z);
    ConduitTarget::new(body, start, end, 2, MARGIN, W_C, w_r, N_SAMPLES)
}

/// The straight route at radius `r0`, nudged in +z (the vertebra's thin axis, the
/// easiest way clear) off the symmetric stationary ridge.
fn off_ridge(t: &ConduitTarget, r0: f64) -> Vec<f64> {
    let mut x = t.x0(r0);
    x[2] += 30.0; // interior CP 0, z
    x[5] += 30.0; // interior CP 1, z
    x
}

/// (1) CLEARANCE INVARIANT: from an off-ridge start whose straight route pierces the
/// bone, the recovered conduit clears it — the whole tube (radius `r` about the
/// centreline) stays outside the vertebra, i.e. `min_clearance ≥ r`, and the centreline
/// is strictly outside (`> 0`). This is the real-anatomy analogue of "the conduit fits".
#[test]
#[ignore = "needs $CF_L4_STL (BodyParts3D FMA13075, CC BY-SA, not committed)"]
fn anatomy_conduit_clears_the_vertebra() {
    let (l4, bc) = load_l4();
    let body = mesh_body(l4, CELL, PAD).expect("grid-cached L4 body");
    let t = conduit(body, bc, W_R);

    // The straight route really does pierce the bone (a genuine detour is required).
    let straight = t.min_clearance(&t.x0(3.0));
    assert!(
        straight < 0.0,
        "straight route should pierce the vertebra (min_clearance {straight:.2} mm)"
    );

    let res = optimize(&t, &off_ridge(&t, 3.0), &t.recommended_config());
    let r = t.radius(&res.params);
    let clr = t.min_clearance(&res.params);
    let req = t.req_clearance(&res.params);
    println!(
        "[clears] r={r:.3} mm  min_clearance={clr:.3} mm  req={req:.3} mm  iters={}  stop={:?}",
        res.iters, res.stop_reason
    );

    assert!(r > 0.0, "log-space radius must stay positive, got {r}");
    assert!(
        clr > 0.0,
        "recovered centreline enters the vertebra: min_clearance {clr:.3} mm"
    );
    assert!(
        clr >= r,
        "the tube (radius {r:.3}) does not clear the bone: min_clearance {clr:.3} mm < r"
    );
}

/// (2) MONOTONE IN THE REWARD: a larger radius reward buys a fatter conduit. Around a
/// compact body in free space there is no clearance ceiling (the route can always
/// detour further), so `r*` grows with `w_r` — a behavioural check that holds on any
/// geometry, replacing the corridor scene's closed-form `r*`. One grid, two rewards.
#[test]
#[ignore = "needs $CF_L4_STL (BodyParts3D FMA13075, CC BY-SA, not committed)"]
fn anatomy_conduit_radius_grows_with_reward() {
    let (l4, bc) = load_l4();
    let body = mesh_body(l4, CELL, PAD).expect("grid-cached L4 body");

    let recover = |w_r: f64| {
        let t = conduit(Arc::clone(&body), bc, w_r);
        let res = optimize(&t, &off_ridge(&t, 3.0), &t.recommended_config());
        t.radius(&res.params)
    };
    let (r_lo, r_hi) = (recover(0.5), recover(3.0));
    println!("[reward] r(w_r=0.5)={r_lo:.3} mm  r(w_r=3.0)={r_hi:.3} mm");

    assert!(
        r_hi > r_lo + 1.0,
        "a larger reward must fatten the conduit: r(3.0)={r_hi:.3} not clearly > r(0.5)={r_lo:.3}"
    );
}

/// (3) GRID-RESOLUTION STABILITY: the recovered design must not be an artefact of the
/// lattice spacing. Halving the cells (finer grid) should move the recovered radius and
/// detour only within a small multiple of the cell — the representation-convergence
/// anchor that stands in for the sphere gate's mesh-vs-analytic cross-check (there is no
/// analytic ground truth for a real vertebra).
#[test]
#[ignore = "needs $CF_L4_STL (BodyParts3D FMA13075, CC BY-SA, not committed)"]
fn anatomy_conduit_stable_across_grid_resolution() {
    let (l4, bc) = load_l4();

    let recover = |cell: f64| {
        let body = mesh_body(l4.clone(), cell, PAD).expect("grid-cached L4 body");
        let t = conduit(body, bc, W_R);
        let res = optimize(&t, &off_ridge(&t, 3.0), &t.recommended_config());
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

    // Measured drift halving the cell is ~0.08 mm radius / ~0.2 mm detour — sub-cell.
    // The bars sit a few multiples above that: tight enough that a resolution-dependent
    // artefact fails, loose enough to absorb the soft equilibrium's limit-cycle jitter.
    assert!(
        (r_fine - r_coarse).abs() < 0.6,
        "recovered radius drifts with grid resolution: {r_fine:.3} (fine) vs {r_coarse:.3} (coarse) mm"
    );
    assert!(
        (d_fine - d_coarse).abs() < 1.5,
        "recovered detour drifts with grid resolution: {d_fine:.3} (fine) vs {d_coarse:.3} (coarse) mm"
    );
}
