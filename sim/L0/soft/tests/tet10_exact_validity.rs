//! Exact validity of the SDF-conformed boundary, on the canonical sphere.
//!
//! [`Tet10Mesh::with_sdf_projected_boundary`] used to sample four Gauss points
//! and call the result valid. Measured on this exact fixture, it shipped
//! **8 of 624 elements folded at a reference corner** while every sampled
//! point stayed positive. It now certifies instead — `det J` bounded over the
//! whole element by its Bernstein coefficients (`element::validity`).
//!
//! The gates here are the ones that measurement earns:
//!
//! - [`the_conformed_sphere_is_certified_element_by_element`] — the property
//!   the old four-point rule could not establish, now asserted directly.
//! - [`conform_quality_floor_sweep`] — the pilot that *chooses* the shipped
//!   floor, printing what each candidate costs in surface conformity. Gate
//!   numbers are learned, not picked.
//! - [`bare_positivity_is_not_a_safe_bar_to_bisect_against`] — the negative
//!   control on the floor itself. Exactness alone would have replaced folded
//!   elements with exactly-degenerate ones, because the back-off lands on
//!   whatever bar it is given.

#![allow(
    // Vertex/tet counts index the `Mesh` trait's `u32` id space; the canonical
    // sphere holds far fewer than `u32::MAX`. Mirrors `tet10_sdf_projection`.
    clippy::cast_possible_truncation,
    // Element counts are exact well below 2^53, and the ratios printed from
    // them are report lines rather than decision thresholds.
    clippy::cast_precision_loss,
    // Scene construction and the Tet10 midside channel surface a meshing or
    // contract failure as a test panic, which is the intent.
    clippy::expect_used
)]

use nalgebra::SMatrix;
use sim_soft::element::{RestValidity, ValidityBar, certify_rest, rest_det_j_coefficients};
use sim_soft::{
    DifferenceSdf, LAYERED_SPHERE_CONFORM_QUALITY_FLOOR, LAYERED_SPHERE_R_CAVITY,
    LAYERED_SPHERE_R_OUTER, MaterialField, Mesh, Sdf, SoftScene, SphereSdf, Tet10Mesh, Vec3,
    VertexId,
};

/// Shear modulus and pressure are immaterial here (no solve); reuse the
/// decision-harness values so the mesh is the one the Lamé gate curves.
const MU: f64 = 2.0e5;
const NU: f64 = 0.4;
const PRESSURE: f64 = 5.0e3;
/// Coarsest canonical cell — a real hollow sphere, cheap to build (no solve).
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

fn straight_sphere_tet10() -> Tet10Mesh {
    let (mesh4, _bc, _initial, _theta) = SoftScene::layered_silicone_sphere(
        MaterialField::uniform(MU, 2.0 * MU * NU / (1.0 - 2.0 * NU)),
        CELL,
        PRESSURE,
    )
    .expect("layered_silicone_sphere meshes at the canonical cell size");
    Tet10Mesh::from_tet4(&mesh4)
}

/// Node coordinates of one element, row per local node.
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

/// Boundary midside vertex ids (trailing three slots of each 6-node face).
fn boundary_midsides(mesh: &Tet10Mesh) -> Vec<VertexId> {
    let mut b: Vec<VertexId> = mesh
        .boundary_faces6()
        .expect("Tet10Mesh surfaces 6-node boundary faces")
        .iter()
        .flat_map(|f| [f[3], f[4], f[5]])
        .collect();
    b.sort_unstable();
    b.dedup();
    b
}

/// How a conformed mesh came out: validity, and what the conformity cost.
struct ConformReport {
    certified: usize,
    violated: usize,
    undetermined: usize,
    /// Worst normalised lower bound on `det J` across the mesh — the exact
    /// bracket's floor, relative to the largest coefficient of that element.
    worst_bound: f64,
    /// Boundary midsides that ended short of the surface (the back-off
    /// engaged), and the largest such shortfall in metres.
    backed_off: usize,
    max_surface_error: f64,
    mean_backed_off_error: f64,
    /// Total boundary midsides, the denominator for `backed_off`.
    boundary_nodes: usize,
}

fn conform_and_report(floor: f64) -> ConformReport {
    let sdf = body_sdf();
    let straight = straight_sphere_tet10();
    let boundary = boundary_midsides(&straight);
    let curved = straight.with_sdf_projected_boundary(&sdf, floor);

    let (mut certified, mut violated, mut undetermined) = (0usize, 0usize, 0usize);
    let mut worst_bound = f64::INFINITY;
    for t in 0..curved.n_tets() as u32 {
        let x_ref = element_nodes(&curved, t);
        match certify_rest(&x_ref, ValidityBar::Positive) {
            RestValidity::Certified { .. } => certified += 1,
            RestValidity::Violated { .. } => violated += 1,
            RestValidity::Undetermined => undetermined += 1,
        }
        let coeffs = rest_det_j_coefficients(&x_ref);
        let lo = coeffs.iter().copied().fold(f64::INFINITY, f64::min);
        let scale = coeffs.iter().fold(0.0f64, |m, c| m.max(c.abs()));
        if scale > 0.0 {
            worst_bound = worst_bound.min(lo / scale);
        }
    }

    let (mut backed_off, mut max_surface_error, mut sum_error) = (0usize, 0.0f64, 0.0f64);
    for &m in &boundary {
        let err = sdf.eval(curved.positions()[m as usize].into()).abs();
        // The projector's own Newton tolerance is 1e-12; anything above it is
        // a node the back-off held short of the surface.
        if err > 1e-9 {
            backed_off += 1;
            max_surface_error = max_surface_error.max(err);
            sum_error += err;
        }
    }

    ConformReport {
        certified,
        violated,
        undetermined,
        worst_bound,
        backed_off,
        max_surface_error,
        mean_backed_off_error: if backed_off == 0 {
            0.0
        } else {
            sum_error / backed_off as f64
        },
        boundary_nodes: boundary.len(),
    }
}

/// ★ The property the four-point rule could not establish: every element of
/// the shipped conformed sphere is *proven* valid over its whole volume.
#[test]
fn the_conformed_sphere_is_certified_element_by_element() {
    let r = conform_and_report(LAYERED_SPHERE_CONFORM_QUALITY_FLOOR);
    println!(
        "conformed sphere @ floor {LAYERED_SPHERE_CONFORM_QUALITY_FLOOR}: \
         {} certified, {} violated, {} undetermined, worst bound {:.4}",
        r.certified, r.violated, r.undetermined, r.worst_bound
    );
    assert_eq!(
        (r.violated, r.undetermined),
        (0, 0),
        "the conformed sphere must be certified element by element; \
         {} violated and {} undetermined",
        r.violated,
        r.undetermined,
    );
    assert!(
        r.certified > 100,
        "non-vacuous: the sphere must have a substantial element count, got {}",
        r.certified,
    );
}

/// ★★ The property the projector actually promises, which is **stronger than
/// "nothing is folded"**: every element clears the quality floor against its
/// own pre-conform geometry, over its whole volume.
///
/// The sibling gate above checks bare positivity, which would still pass if
/// the sweep's invariant were broken — a node's placement only re-checks the
/// elements incident to *that* node, so "every accepted move leaves every
/// incident element above the floor" is an induction over the sweep, not
/// something any single predicate call establishes. This gate checks the
/// conclusion of that induction directly, on the finished mesh, against the
/// straight mesh it started from.
///
/// ⚠ **Tested at `floor · (1 − 1e-6)`, and the relaxation is the point, not a
/// fudge.** The back-off bisects to the furthest blend the floor admits, so it
/// converges *onto* the bar: `det J_c − floor · det J_o` is driven to zero by
/// construction, and the tightest element certifies with a relative margin of
/// ~1.08e-12 against the primitive's own 1e-12 noise threshold. Asserting the
/// exact floor would therefore be a knife-edge that a different CPU or
/// optimisation level could tip, and it would read as a mesher regression when
/// it was only rounding. The relaxation is far below the bisection's own
/// resolution (`2⁻⁴⁰ ≈ 9e-13` in the blend parameter), so nothing real can
/// hide inside it.
#[test]
fn every_element_of_the_conformed_sphere_clears_the_floor_not_merely_zero() {
    /// Slack for the bisection converging onto the bar — see the note above.
    const BAR: f64 = LAYERED_SPHERE_CONFORM_QUALITY_FLOOR * (1.0 - 1e-6);

    let straight = straight_sphere_tet10();
    let curved = straight
        .clone()
        .with_sdf_projected_boundary(&body_sdf(), LAYERED_SPHERE_CONFORM_QUALITY_FLOOR);
    assert_eq!(
        straight.n_tets(),
        curved.n_tets(),
        "conforming must not change topology"
    );

    // Collected rather than panicked on the first: if the sweep's induction is
    // broken, how MANY elements it left short is the diagnostic, not which one
    // happened to come first.
    let mut short = Vec::new();
    let mut worst_margin = f64::INFINITY;
    for t in 0..curved.n_tets() as u32 {
        let before = element_nodes(&straight, t);
        let after = element_nodes(&curved, t);
        match certify_rest(
            &after,
            ValidityBar::RelativeFloor {
                original: &before,
                floor: BAR,
            },
        ) {
            RestValidity::Certified { margin } => worst_margin = worst_margin.min(margin),
            other => short.push((t, other)),
        }
    }
    assert!(
        short.is_empty(),
        "{} of {} elements do not clear the floor they were conformed at — the sweep's \
         induction is broken, not merely its positivity. First few: {:?}",
        short.len(),
        curved.n_tets(),
        &short[..short.len().min(5)],
    );
    println!(
        "every element clears {BAR} (floor {LAYERED_SPHERE_CONFORM_QUALITY_FLOOR} less \
         bisection slack); tightest relative margin {worst_margin:.3e}"
    );
    // Non-vacuous in the other direction: the back-off really did converge
    // onto the bar, so this gate is measuring a tight fit rather than a mesh
    // that never approached the floor at all.
    assert!(
        worst_margin < 1e-5,
        "the back-off should land ON the bar; tightest margin {worst_margin:e} suggests it \
         never engaged, which would make this gate vacuous"
    );
}

/// The pilot that chooses [`LAYERED_SPHERE_CONFORM_QUALITY_FLOOR`].
///
/// Prints, per candidate floor: how many elements certify, the worst
/// normalised `det J` bound anywhere in the mesh, and what the floor costs in
/// surface conformity (nodes held short of the SDF, and by how far). The
/// shipped value is read off this table, not chosen and then justified.
#[test]
fn conform_quality_floor_sweep() {
    println!(
        "\n{:>6} {:>10} {:>9} {:>13} {:>11} {:>14}",
        "floor", "certified", "violated", "undetermined", "worst bound", "backed off"
    );
    for &floor in &[0.0, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.7] {
        let r = conform_and_report(floor);
        println!(
            "{floor:>6.2} {:>10} {:>9} {:>13} {:>11.6} {:>4}/{} (max {:.2e} m, mean {:.2e} m)",
            r.certified,
            r.violated,
            r.undetermined,
            r.worst_bound,
            r.backed_off,
            r.boundary_nodes,
            r.max_surface_error,
            r.mean_backed_off_error,
        );
    }
    println!(
        "  'worst bound' is the exact lower bound on det J over the WHOLE mesh, normalised\n  \
         per element. 'backed off' counts boundary midsides the floor held short of the\n  \
         surface — the conformity the floor costs."
    );
}

/// ⚠ Negative control on the *floor*, not on the certifier.
///
/// Exactness alone is not the fix. The back-off bisects to the furthest
/// feasible blend, so it lands on whatever bar it is given — and bare
/// positivity is the `det J → 0⁺` boundary. Certifying against `floor = 0`
/// must therefore leave the mesh measurably closer to degenerate than the
/// shipped floor does, or the floor argument is decoration.
#[test]
fn bare_positivity_is_not_a_safe_bar_to_bisect_against() {
    let bare = conform_and_report(0.0);
    let floored = conform_and_report(LAYERED_SPHERE_CONFORM_QUALITY_FLOOR);

    println!(
        "worst normalised det J bound — floor 0.0: {:.6}, floor {}: {:.6}",
        bare.worst_bound, LAYERED_SPHERE_CONFORM_QUALITY_FLOOR, floored.worst_bound
    );

    // Both are sound: an exact certificate never ships a folded element,
    // whatever the floor.
    assert_eq!(
        (bare.violated, bare.undetermined),
        (0, 0),
        "exact certification is sound at any floor"
    );
    // But bare positivity walks the mesh right up to degeneracy, and the floor
    // is what stops it.
    assert!(
        floored.worst_bound > bare.worst_bound,
        "the quality floor must hold the mesh further from degeneracy than bare \
         positivity does; got {:.6} vs {:.6}",
        floored.worst_bound,
        bare.worst_bound,
    );
}
