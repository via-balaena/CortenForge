//! Gate for routing a [`ConduitTarget`] against a **mesh** body rather than an
//! analytic [`Solid`] — the geometry axis's real-body rung.
//!
//! The scene is a unit sphere at the origin, imported through the mesh pipeline
//! ([`solid_mesh_body`]: mesh the sphere, then cache a trilinear signed-distance
//! grid). A sphere is chosen because its exact field is known in closed form
//! (`‖p‖ − R`), which turns "is the gridded mesh body faithful?" into numbers, and
//! because meshing *the same* `Solid::sphere` as an analytic body gives a
//! representation-only cross-check: the two bodies are identical geometry, so a route
//! optimized against each must land in the same place up to the grid's discretization.
//!
//! What replaces the corridor scene's closed-form `r*`: on curved anatomy neither the
//! binding-sample count `m` nor the mean binding clearance `φ̄` is known, so the
//! quantitative `r* = (φ̄ − margin) + w_r/(2·m·w_c)` prediction does not transfer. The
//! falsifiable anchors here are instead (1) the mesh field matches the analytic sphere
//! to well within the grid cell, (2) the target's finite-difference gradient is stable
//! across a 100× change of step (the field is smooth at the FD scale, not kink-noise),
//! and (3) a conduit recovered against the mesh body both *clears* it and matches the
//! design recovered against the same body taken analytically.
//!
//! The analytic-scene gates in `conduit_inverse_design.rs` are unchanged — they remain
//! the quantitative anchor; this file *adds* the mesh scene.

use std::sync::{Arc, LazyLock};

use cf_codesign::{CoDesignProblem, ConduitTarget, optimize, solid_mesh_body};
use cf_design::{Sdf, Solid};
use nalgebra::Point3;

const R: f64 = 0.6; // sphere radius
const MESH_TOL: f64 = 0.03; // marching-cubes cell for the sphere mesh
const CELL: f64 = 0.05; // signed-distance grid spacing
// Grid padding beyond the mesh AABB. It must cover the *whole route domain*, not
// just the body: the fixed endpoints at ±END and the +y detour both have to fall
// inside the lattice, or an out-of-grid query returns a clamped (under-reported)
// distance and corrupts the clearance the optimizer sees. Here `AABB(±0.6) + 1.0`
// gives ±1.6, comfortably enclosing the ±1.3 endpoints and the ~1.0 detour.
const PAD: f64 = 1.0;
const END: f64 = 1.3; // route endpoints at (±END, 0, 0)

const MARGIN: f64 = 0.1;
const W_C: f64 = 10.0; // clearance penalty weight
const W_R: f64 = 1.0; // radius reward — modest, so the conduit stays near the body
const N_SAMPLES: usize = 40;

/// The sphere as a grid-cached mesh body — built once and shared across the gate's
/// tests, since the grid fill is the only expensive step (queries are O(1)).
static MESH_BODY: LazyLock<Arc<dyn Sdf>> = LazyLock::new(|| {
    solid_mesh_body(&Solid::sphere(R), MESH_TOL, CELL, PAD).expect("sphere mesh body builds")
});

/// The same sphere as an analytic body — the exact-field control for the cross-check.
fn analytic_body() -> Arc<dyn Sdf> {
    Arc::new(Solid::sphere(R))
}

/// Closed-form signed distance to the sphere surface (negative inside).
fn sphere_sdf(p: Point3<f64>) -> f64 {
    p.coords.norm() - R
}

/// A conduit problem routing `body` from (-END,0,0) to (END,0,0) — a straight line
/// that passes through the sphere's center, so a feasible conduit must detour.
fn scene(body: Arc<dyn Sdf>) -> ConduitTarget {
    ConduitTarget::new(
        body,
        Point3::new(-END, 0.0, 0.0),
        Point3::new(END, 0.0, 0.0),
        2,
        MARGIN,
        W_C,
        W_R,
        N_SAMPLES,
    )
}

/// The straight route at radius `r0`, nudged in +y off the symmetric stationary ridge.
fn off_ridge(t: &ConduitTarget, r0: f64, nudge: f64) -> Vec<f64> {
    let mut x = t.x0(r0);
    x[1] += nudge; // interior CP 0, y
    x[4] += nudge; // interior CP 1, y
    x
}

/// (1) The gridded mesh body reproduces the analytic sphere field across the
/// clearance-relevant band (a shell just outside the surface, where the penalty
/// acts). Deviation must be a small fraction of the grid cell — and orders below the
/// `req ≈ 0.4` clearances the optimizer reasons about, so no clearance decision can
/// flip on discretization error.
#[test]
fn mesh_body_field_matches_analytic_sphere() {
    let body = MESH_BODY.clone();
    let mut max_err = 0.0_f64;
    for i in 0..8 {
        for j in 0..8 {
            let th = std::f64::consts::PI * (i as f64 + 0.5) / 8.0;
            let ph = 2.0 * std::f64::consts::PI * (j as f64) / 8.0;
            let dir = Point3::new(th.sin() * ph.cos(), th.sin() * ph.sin(), th.cos());
            // Probe at R + 0.3 — inside the padded grid, in the clearance band.
            let p = Point3::new(dir.x * (R + 0.3), dir.y * (R + 0.3), dir.z * (R + 0.3));
            max_err = max_err.max((body.eval(p) - sphere_sdf(p)).abs());
        }
    }
    // Measured ~1.0e-3 for this scene; the bar sits an order below the grid cell
    // (0.05) and two below the working clearances (~0.4), so no clearance decision can
    // flip on it, while staying tight enough (~2× the measured deviation) to fail if
    // the pipeline breaks.
    assert!(
        max_err < 2.0e-3,
        "mesh SDF deviates from the analytic sphere by {max_err} (bar 2e-3)"
    );
}

/// (2) The target's finite-difference gradient over the mesh body is *stable*: the
/// internal stencil (eps 1e-6) agrees with an independent central FD at eps 1e-4, a
/// 100× larger step. Agreement across that range is the falsifiable form of "the
/// trilinear field is smooth at the FD scale" — if facet/cell kinks dominated, the two
/// steps would disagree. Looser than the analytic sibling's `1e-3` bar because the
/// gridded field is genuinely less smooth than a primitive's closed form.
#[test]
fn conduit_over_mesh_body_gradient_is_fd_stable() {
    let t = scene(MESH_BODY.clone());
    let x = off_ridge(&t, 0.3, 0.15);
    let (_loss, grad) = t.evaluate(&x);
    assert_eq!(grad.len(), 7, "6 coordinates + ln r");

    let eps = 1e-4; // 100× the target's internal 1e-6
    for i in 0..grad.len() {
        let mut xp = x.clone();
        xp[i] += eps;
        let jp = t.objective(&xp);
        xp[i] -= 2.0 * eps;
        let jm = t.objective(&xp);
        let fd = (jp - jm) / (2.0 * eps);
        // Measured rel disagreement is ~1e-6 (the field is smooth at the FD scale);
        // facet/cell kinks would push this to ~1e-2. The bar sits decisively between,
        // with ample headroom over the measurement for cross-platform FP.
        let tol = 1.0e-4 * grad[i].abs().max(1.0);
        assert!(
            (grad[i] - fd).abs() < tol,
            "param {i}: internal grad {} vs independent FD {fd} (tol {tol})",
            grad[i]
        );
    }
}

/// (3) A conduit recovered against the mesh body (a) clears it and (b) matches the
/// design recovered against the *same sphere taken analytically* — the ground-truth
/// anchor that replaces the corridor's closed-form radius. Same geometry, so the two
/// representations must converge to nearly the same route and radius, up to the grid's
/// discretization.
#[test]
fn conduit_over_mesh_body_clears_and_tracks_analytic() {
    let mesh_t = scene(MESH_BODY.clone());
    let ana_t = scene(analytic_body());
    let x0 = off_ridge(&mesh_t, 0.3, 0.5);

    let rm = optimize(&mesh_t, &x0, &mesh_t.recommended_config());
    let ra = optimize(&ana_t, &x0, &ana_t.recommended_config());

    // (a) The recovered conduit clears the mesh body: the tightest centerline sample
    // stays outside the sphere with room for the tube. The soft equilibrium sits a
    // slack `w_r/(2·m·w_c)` inside `req`, so the bar allows that slack while still
    // proving the centerline never enters the body (min_clearance > 0).
    let clr = mesh_t.min_clearance(&rm.params);
    let req = mesh_t.req_clearance(&rm.params);
    assert!(
        clr > 0.0,
        "recovered centerline enters the body: min_clearance {clr} ≤ 0"
    );
    let slack = W_R / (2.0 * W_C); // single-binding-sample upper bound on the shortfall
    assert!(
        clr >= req - slack,
        "conduit under-clears: min_clearance {clr} < req {req} − slack {slack}"
    );

    // (b) Radius and detour track the analytic-body recovery. The two recoveries land
    // on the same design to well under a grid cell (measured ~2e-4 radius, ~7e-4
    // detour); the bars sit ~10× above that, far below the cell (0.05) so they still
    // certify a representation-faithful recovery, yet loose enough for Adam's
    // cross-platform FP.
    let (r_mesh, r_ana) = (mesh_t.radius(&rm.params), ana_t.radius(&ra.params));
    assert!(
        (r_mesh - r_ana).abs() < 2.0e-3,
        "recovered radius {r_mesh} (mesh) vs {r_ana} (analytic) differ by more than 2e-3"
    );
    // The detour is the interior control points' y; compare their mean magnitude.
    let detour_mesh = (rm.params[1].abs() + rm.params[4].abs()) / 2.0;
    let detour_ana = (ra.params[1].abs() + ra.params[4].abs()) / 2.0;
    assert!(
        (detour_mesh - detour_ana).abs() < 5.0e-3,
        "recovered detour {detour_mesh} (mesh) vs {detour_ana} (analytic) differ by more than 5e-3"
    );
}
