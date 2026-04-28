//! III-2 — Quality bounds on the parametric sphere family.
//!
//! Sweeps [`SdfMeshedTetMesh::from_sdf`] over `r ∈ {0.05, 0.1, 0.2}` ×
//! `cell_size ∈ {r/3, r/5, r/8}` (9 combinations) and asserts four
//! quality contracts against the post-degenerate-filter mesh:
//!
//! 1. Every emitted sub-tet has `signed_volume > 0` strict — the
//!    structural detector for the D-10 degenerate filter and the
//!    post-hoc orientation swap (scope memo §3 Decision M, Decision
//!    H). A topology bug emitting a wrong vertex set surfaces as a
//!    near-zero or negative volume; a bug producing left-handed sub-
//!    tets that the swap silently corrects would leak through this
//!    assertion (the swap preserves the four-vertex set), so the
//!    strict-positivity bar is paired with the aspect-ratio and
//!    dihedral bounds — together they triangulate the contract.
//! 2. Every tet has `aspect_ratio ≥ RHO_MIN` — the sliver detector.
//!    Under correct BCC + warp + stuffing operation, Labelle-
//!    Shewchuk Theorem 1 with the pinned "min dihedral, safe"
//!    parameters guarantees `aspect_ratio ≥ ≈0.08`; this assertion
//!    catches any departure from the published algorithm (warp
//!    threshold drift, stencil-dispatch bug).
//! 3. Every tet has `dihedral_min ≥ ALPHA_LO` and `dihedral_max ≤
//!    ALPHA_HI` — the dihedral-tail detector. Theorem 1 guarantees
//!    dihedral angles in `[9.3171°, 161.6432°]`; aspect ratio alone
//!    misses some sliver shapes, the dihedral pair covers them.
//! 4. At each fixed `cells_per_radius`, total mesh volume is
//!    strictly monotone increasing with `r`: `vol(0.05) < vol(0.1)
//!    < vol(0.2)`. Catches a degenerate mesher that produces
//!    constant volume regardless of `r`, and catches any pathological
//!    case where total volume DECREASES with larger radius. Cell
//!    size scales with `r` so each mesh has roughly the same
//!    boundary-tet density relative to its sphere; the comparison
//!    stays meaningful across the sweep.
//!
//! Memo-locked sanity floor (scope memo §1 III-2 + §3 Decision B):
//! `RHO_MIN ≥ 0.05`, `ALPHA_LO ≥ 5°`, `ALPHA_HI ≤ 175°`. Under
//! correct BCC + warp + stuffing operation the floor is cleared by
//! Labelle-Shewchuk Theorem 1 (paper §4.1, computer-verified to
//! 0.0001°) with the pinned "min dihedral, safe" parameters
//! (`α_long = 0.24999`, `α_short = 0.41189` per scope memo §3
//! Decision A): dihedral angles in `[9.3171°, 161.6432°]`, aspect
//! ratios `≥ ≈0.08` — margins 4.32°/13.36°/0.03 above the floor.
//! If observed values land WORSE than the floor, the implementation
//! has departed from the published algorithm — abort and investigate
//! (warp threshold drift, stencil-dispatch bug, D-10 backstop
//! firing). The floor is theorem-implied, not parameter-tunable;
//! relaxing the thresholds would mask a structural regression.
//!
//! The empirical thresholds named below sit at the "loosest within
//! floor" position per scope memo §9 (resolves at commit 7) — they
//! ride the sanity floor itself, leaving the recorded margin to
//! Theorem 1's guarantee. `report_observed_quality_extrema` prints
//! the actual mins/maxes so future Phase H tightening has a
//! baseline. BCC interior tets (post-warp, untouched by stencil
//! dispatch) are uniform Sommerville with closed-form geometry
//! comfortably above the floor; sub-tets emitted by stencil
//! dispatch at non-trivial sign cases are the floor-driving
//! population — Theorem 1 bounds these by construction.
//!
//! Optional III-4 (volume convergence to `(4/3)πr³`) is folded in as
//! `volume_relative_error_decreases_monotonically_with_resolution` —
//! recommend-first per `feedback_recommend_first_deep_specialist`.
//! The III-2 sweep already builds the meshes the convergence test
//! needs, so adding the assertion is essentially free.

// `build_sphere_mesh` calls `.expect()` to surface meshing failures
// as test panics — that IS the test failure signal under the same
// inline-test convention as III-1's `sdf_pipeline_determinism`.
#![allow(clippy::expect_used)]

use sim_soft::sdf_bridge::{Aabb3, MeshingHints, SdfMeshedTetMesh, SphereSdf};
use sim_soft::{MaterialField, Mesh, Vec3};

/// Sphere radii (m) — 4× span between smallest and largest.
const RADII: [f64; 3] = [0.05, 0.1, 0.2];

/// Cells-per-radius factors — `cell_size = r / N`. The `N=3` value
/// is the coarse end (~6 cells across the sphere diameter), `N=8`
/// is the fine end (~16 cells), and `N=5` sits between them.
const CELLS_PER_RADIUS: [f64; 3] = [3.0, 5.0, 8.0];

/// Bounding-box half-extent multiplier — `bbox = [-α·r, α·r]³`.
/// `1.2` matches III-1's bbox sizing and gives a thin clearance
/// band of unsigned cells around the sphere.
const BBOX_MARGIN: f64 = 1.2;

// Empirical thresholds — see file-level docstring. Sit within the
// memo-locked sanity floor (RHO_MIN ≥ 0.05, ALPHA_LO ≥ 5°, ALPHA_HI
// ≤ 175°). Picked at "loosest within floor" — the goal is a future-
// phase regression detector, not an upper-bound on current quality.
// `report_observed_quality_extrema` prints the actual extrema so
// Phase H tightening has a recorded baseline.
const RHO_MIN: f64 = 0.05;
const ALPHA_LO_DEG: f64 = 5.0;
const ALPHA_HI_DEG: f64 = 175.0;

/// Optional III-4 soft-bound — relative volume error to `(4/3)πr³`
/// at the finest resolution (`cell_size = r/8`). Empirically
/// observed for the BCC + warp + stuffing pipeline; not load-bearing
/// per scope memo §1 III-4.
const VOLUME_REL_ERROR_BOUND_AT_R_OVER_8: f64 = 0.15;

/// Build a sphere mesh at the given `(radius, cell_size)` parameters.
fn build_sphere_mesh(radius: f64, cell_size: f64) -> SdfMeshedTetMesh {
    let half = BBOX_MARGIN * radius;
    let hints = MeshingHints {
        bbox: Aabb3::new(Vec3::new(-half, -half, -half), Vec3::new(half, half, half)),
        cell_size,
    };
    SdfMeshedTetMesh::from_sdf(
        &SphereSdf { radius },
        &hints,
        &MaterialField::uniform(1.0e5, 4.0e5),
    )
    .expect("sphere scene should mesh successfully")
}

/// One row of the `RADII × CELLS_PER_RADIUS` sweep — `(radius,
/// cell_size, mesh, total_signed_volume)`.
struct Combo {
    radius: f64,
    cell_size: f64,
    mesh: SdfMeshedTetMesh,
    total_volume: f64,
}

/// Build all 9 sweep combinations. Sweep order: outer `r`, inner
/// `cells_per_radius`. Each test rebuilds — `from_sdf` is ~10 ms per
/// call, so 9 × ~10 ms = ~90 ms × 6 tests ≈ 0.5 s, well inside
/// budget.
fn sweep() -> Vec<Combo> {
    let mut out = Vec::with_capacity(9);
    for &r in &RADII {
        for &n in &CELLS_PER_RADIUS {
            let cell_size = r / n;
            let mesh = build_sphere_mesh(r, cell_size);
            let total: f64 = mesh.quality().signed_volume.iter().sum();
            out.push(Combo {
                radius: r,
                cell_size,
                mesh,
                total_volume: total,
            });
        }
    }
    out
}

#[test]
fn report_observed_quality_extrema() {
    // Diagnostic — surfaces the actual observed mins/maxes via
    // eprintln so `cargo test --nocapture` shows the empirical
    // headroom above the chosen thresholds. Always green; the
    // record IS the deliverable for future Phase H tightening.
    eprintln!();
    eprintln!("III-2 observed quality extrema across 9-combination sphere sweep");
    eprintln!("------------------------------------------------------------------");
    let mut global_min_aspect = f64::INFINITY;
    let mut global_min_dihedral = f64::INFINITY;
    let mut global_max_dihedral = f64::NEG_INFINITY;
    let mut global_min_volume = f64::INFINITY;
    for combo in sweep() {
        let q = combo.mesh.quality();
        let n = q.signed_volume.len();
        let local_min_aspect = q.aspect_ratio.iter().copied().fold(f64::INFINITY, f64::min);
        let local_min_dihedral = q.dihedral_min.iter().copied().fold(f64::INFINITY, f64::min);
        let local_max_dihedral = q
            .dihedral_max
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        let local_min_volume = q
            .signed_volume
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        eprintln!(
            "  r={:.2} cell={:.4} ntet={:5} aspect_min={:.4} dih_min={:6.2}° dih_max={:6.2}° vol_min={:.3e} total={:.4e}",
            combo.radius,
            combo.cell_size,
            n,
            local_min_aspect,
            local_min_dihedral.to_degrees(),
            local_max_dihedral.to_degrees(),
            local_min_volume,
            combo.total_volume,
        );
        global_min_aspect = global_min_aspect.min(local_min_aspect);
        global_min_dihedral = global_min_dihedral.min(local_min_dihedral);
        global_max_dihedral = global_max_dihedral.max(local_max_dihedral);
        global_min_volume = global_min_volume.min(local_min_volume);
    }
    eprintln!("------------------------------------------------------------------");
    eprintln!(
        "  GLOBAL aspect_min={:.4} dih_min={:.2}° dih_max={:.2}° vol_min={:.3e}",
        global_min_aspect,
        global_min_dihedral.to_degrees(),
        global_max_dihedral.to_degrees(),
        global_min_volume,
    );
    eprintln!(
        "  thresholds RHO_MIN={RHO_MIN:.3} ALPHA_LO={ALPHA_LO_DEG:.1}° ALPHA_HI={ALPHA_HI_DEG:.1}°",
    );
    eprintln!();
}

#[test]
fn signed_volume_is_strictly_positive_for_every_emitted_tet() {
    for combo in sweep() {
        for (tet_id, &v) in combo.mesh.quality().signed_volume.iter().enumerate() {
            assert!(
                v > 0.0,
                "r={} cell_size={} tet {tet_id}: signed_volume {v:e} not strictly positive — \
                 D-10 filter or post-hoc orientation swap regressed",
                combo.radius,
                combo.cell_size,
            );
        }
    }
}

#[test]
fn aspect_ratio_clears_sanity_floor_for_every_tet() {
    for combo in sweep() {
        for (tet_id, &a) in combo.mesh.quality().aspect_ratio.iter().enumerate() {
            assert!(
                a >= RHO_MIN,
                "r={} cell_size={} tet {tet_id}: aspect_ratio {a:.4} below RHO_MIN {RHO_MIN}",
                combo.radius,
                combo.cell_size,
            );
        }
    }
}

#[test]
fn dihedral_angles_stay_within_sanity_floor_for_every_tet() {
    let alpha_lo_rad = ALPHA_LO_DEG.to_radians();
    let alpha_hi_rad = ALPHA_HI_DEG.to_radians();
    for combo in sweep() {
        let q = combo.mesh.quality();
        for tet_id in 0..q.dihedral_min.len() {
            let lo = q.dihedral_min[tet_id];
            let hi = q.dihedral_max[tet_id];
            assert!(
                lo >= alpha_lo_rad,
                "r={} cell_size={} tet {tet_id}: dihedral_min {:.2}° below ALPHA_LO {ALPHA_LO_DEG}°",
                combo.radius,
                combo.cell_size,
                lo.to_degrees(),
            );
            assert!(
                hi <= alpha_hi_rad,
                "r={} cell_size={} tet {tet_id}: dihedral_max {:.2}° above ALPHA_HI {ALPHA_HI_DEG}°",
                combo.radius,
                combo.cell_size,
                hi.to_degrees(),
            );
        }
    }
}

#[test]
fn total_volume_increases_strictly_with_radius_at_fixed_cells_per_radius() {
    // For each cells-per-radius factor N, the chain
    //   vol(r=0.05, cell_size=0.05/N)
    //   < vol(r=0.10, cell_size=0.10/N)
    //   < vol(r=0.20, cell_size=0.20/N)
    // must hold strictly. Cell size scales with r, so each mesh
    // has roughly the same boundary-tet density relative to its
    // sphere — a degenerate mesher producing constant volume
    // regardless of r is caught by either chain step, and so is
    // any pathological case where volume decreases with larger r.
    let combos = sweep();
    for (i, &n) in CELLS_PER_RADIUS.iter().enumerate() {
        // Sweep order is outer r, inner cells_per_radius — so the
        // three combos at factor index `i` are at indices i, 3+i,
        // 6+i.
        let v_small = combos[i].total_volume;
        let v_mid = combos[3 + i].total_volume;
        let v_large = combos[6 + i].total_volume;
        assert!(
            v_small < v_mid,
            "cells_per_radius={n}: vol(r=0.05)={v_small:.4e} not < vol(r=0.10)={v_mid:.4e}",
        );
        assert!(
            v_mid < v_large,
            "cells_per_radius={n}: vol(r=0.10)={v_mid:.4e} not < vol(r=0.20)={v_large:.4e}",
        );
    }
}

#[test]
fn volume_relative_error_decreases_monotonically_with_resolution() {
    // Optional III-4 — volume convergence to the analytic
    // `(4/3)πr³` as `cell_size → 0`. NOT load-bearing; recommend-
    // first per `feedback_recommend_first_deep_specialist`. Folded
    // into commit 6 because the III-2 sweep already builds the
    // meshes this needs.
    //
    // Anchored on `r = 0.1` (the III-1 / III-3 canonical radius).
    // At three resolutions `cell_size ∈ {r/3, r/5, r/8}`, relative
    // error to `(4/3)π(0.1)³` is expected to decrease monotonically.
    // The soft-bound `≤ VOLUME_REL_ERROR_BOUND_AT_R_OVER_8` at the
    // finest resolution catches a mesher that converges to the
    // wrong volume.
    let radius = 0.1_f64;
    let analytic = 4.0 / 3.0 * std::f64::consts::PI * radius.powi(3);
    let mut prev_rel = f64::INFINITY;
    let mut finest_rel = f64::NAN;
    for &n in &CELLS_PER_RADIUS {
        let cell_size = radius / n;
        let mesh = build_sphere_mesh(radius, cell_size);
        let total: f64 = mesh.quality().signed_volume.iter().sum();
        let rel = (total - analytic).abs() / analytic;
        eprintln!(
            "III-4 r=0.1 cells_per_radius={n} cell_size={cell_size:.4} \
             total={total:.6e} analytic={analytic:.6e} rel_err={rel:.4}",
        );
        assert!(
            rel <= prev_rel,
            "relative volume error did not decrease at cells_per_radius={n}: \
             prev={prev_rel:.4} current={rel:.4}",
        );
        prev_rel = rel;
        finest_rel = rel;
    }
    assert!(
        finest_rel <= VOLUME_REL_ERROR_BOUND_AT_R_OVER_8,
        "relative volume error at finest resolution {finest_rel:.4} exceeds soft-bound \
         {VOLUME_REL_ERROR_BOUND_AT_R_OVER_8}",
    );
}
