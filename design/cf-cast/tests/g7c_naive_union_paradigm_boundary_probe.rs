//! §G-7c naive-union paradigm-boundary characterisation — survives
//! the 2026-05-24 FDM-friendly arc post-mortem as a permanent
//! regression guard on the recon-4 (P) framework's CONTAINED /
//! PROTRUDING / WELDED-TO-BULK pattern (see
//! [[project-cf-cast-sdf-meshcsg-paradigm-boundary]]).
//!
//! # Why this test exists
//!
//! The original §G-7 probe at `g7_g9_prismatic_pin_probe.rs`
//! measured `Manifold::union` of a `Manifold::hull_pts` truncated-
//! pyramid onto a synthetic half-sphere shell host and observed
//! BRANCH B (component++) + BRANCH C (SelfIntersecting Critical F4).
//! That finding drove the recon-1 §G-12 #2 SDF-side bail-out
//! decision — which silently undid the prior mating-features arc's
//! load-bearing fix (POST-MC mesh-CSG primitives ARE the right
//! placement for fine mating features per the recon-4 framework;
//! pre-MC SDF composition shares the bulk MC cell size with the
//! features → features sub-MC-cell get eaten by quantization).
//!
//! What the §G-7 probe MISSED: a cylinder control on the same
//! synthetic shell host. This test runs that control. Result
//! (2026-05-24): cylinder mesh-CSG produces the SAME BRANCH B+C
//! symptoms on the synthetic shell. The §G-7 "evidence against
//! truncated-pyramid" was actually evidence against any naive
//! `Manifold::union` of a primitive onto an MC-derived curved-shell
//! host — NOT shape-specific.
//!
//! # What "naive" means here
//!
//! Naive = pose-undisciplined: arbitrary primitive position /
//! orientation chosen for probe convenience, NOT derived from the
//! ribbon's binormal axis / annulus-midpoint pin center. Production
//! mesh-CSG pins (S5 mating-features arc, shipped in PR #255 as
//! `aadcfed6`) come through `apply_mating_transforms` with poses
//! derived from `build_registration_transforms` — cylinder axis =
//! binormal (perpendicular to seam plane), center = cup-wall
//! annulus midpoint, half-length straddles the seam plane
//! symmetrically. This gives the CONTAINED / PROTRUDING pattern
//! the recon-4 framework requires for clean mesh-CSG boolean
//! welding.
//!
//! # Conclusion
//!
//! Any future recon that proposes "move primitive emission from
//! mesh-CSG to SDF because synthetic-shell mesh-CSG union fails"
//! must FIRST add a control of the new primitive on the same
//! synthetic host. If the control fails the same way, the
//! synthetic-shell evidence is non-discriminating; the actual
//! Production-pipeline behaviour gates the decision. See
//! [[feedback-read-prior-arc-memory-before-architectural-decisions]].

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::doc_markdown,
    clippy::redundant_closure_for_method_calls,
    clippy::uninlined_format_args,
    dead_code
)]

use cf_design::Solid;
use manifold3d::Manifold;
use mesh_offset::{MarchingCubesConfig, ScalarGrid, marching_cubes};
use mesh_printability::{IssueSeverity, PrintIssueType, PrinterConfig, validate_for_printing};
use mesh_repair::components::find_connected_components;
use mesh_types::IndexedMesh;
use nalgebra::Vector3;

const METERS_TO_MM: f64 = 1000.0;
const HOST_GRID_PADDING_CELLS: usize = 2;

const SHELL_BODY_RADIUS_M: f64 = 0.030;
const SHELL_HALF_THICKNESS_M: f64 = 0.005;
const PROD_CELL_SIZE_M: f64 = 0.003;

const PIN_RADIUS_OFFSET_MM: f64 = 32.5;
const PIN_Z_EQUATOR_MM: f64 = 5.0;
const CYL_RADIUS_MM: f64 = 1.5;
const CYL_HALF_LENGTH_MM: f64 = 3.0;
const CYL_SEGMENTS: i32 = 32;

/// Build the same synthetic half-sphere shell host the §G-7 probe
/// uses (mirrors `g7_g9_prismatic_pin_probe.rs:177-194`).
fn build_synthetic_shell_host_manifold(cell_size_m: f64) -> Manifold {
    let sphere = Solid::sphere(SHELL_BODY_RADIUS_M);
    let shell = sphere.shell(SHELL_HALF_THICKNESS_M);
    let upper_halfspace = Solid::plane(Vector3::new(0.0, 0.0, -1.0), 0.0);
    let half_shell = shell.intersect(upper_halfspace);

    let bounds = half_shell.bounds().expect("synth shell has finite bounds");
    let mut grid =
        ScalarGrid::from_bounds(bounds.min, bounds.max, cell_size_m, HOST_GRID_PADDING_CELLS);
    let (nx, ny, nz) = grid.dimensions();
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let p = grid.position(ix, iy, iz);
                grid.set(ix, iy, iz, half_shell.evaluate(&p));
            }
        }
    }
    let mut mesh = marching_cubes(&grid, &MarchingCubesConfig::default());
    for v in &mut mesh.vertices {
        v.x *= METERS_TO_MM;
        v.y *= METERS_TO_MM;
        v.z *= METERS_TO_MM;
    }
    Manifold::from_vertices_and_faces(&mesh.vertices, &mesh.faces)
        .expect("synth shell MC output is manifold-clean")
}

fn manifold_to_indexed_mesh(m: &Manifold) -> IndexedMesh {
    let (verts, faces) = m.to_vertices_and_faces();
    let vertices: Vec<mesh_types::Point3<f64>> = verts
        .into_iter()
        .map(|p| mesh_types::Point3::new(p.x, p.y, p.z))
        .collect();
    IndexedMesh::from_parts(vertices, faces)
}

fn critical_issue_types(mesh: &IndexedMesh) -> Vec<PrintIssueType> {
    let validation = validate_for_printing(mesh, &PrinterConfig::fdm_default())
        .expect("non-empty mesh validates without error");
    let mut types: Vec<PrintIssueType> = validation
        .issues
        .iter()
        .filter(|i| i.severity == IssueSeverity::Critical)
        .map(|i| i.issue_type)
        .collect();
    types.sort_by_key(|t| t.as_str());
    types.dedup();
    types
}

/// THE LOAD-BEARING CONTROL: cylinder mesh-CSG `Manifold::union`
/// onto the synthetic §G-7 shell host (naive pose: +Z axis, at the
/// same wall midpoint the §G-7 hull_pts pyramid uses) produces the
/// SAME BRANCH B (post-components > baseline) + BRANCH C
/// (SelfIntersecting added) symptoms as the §G-7 hull_pts probe.
///
/// This refutes the implicit "evidence against truncated-pyramid"
/// reading of the §G-7 probe. Naive `Manifold::union` of ANY
/// primitive onto a MC-derived curved-shell host produces the
/// disconnection / SelfIntersecting symptoms — it's not a property
/// of hull_pts pyramid specifically. The discriminating variable
/// is pose discipline (`apply_mating_transforms` with binormal axis,
/// annulus-midpoint center, symmetric-across-seam placement) per
/// the recon-4 (P) CONTAINED / PROTRUDING / WELDED-TO-BULK framework.
#[test]
fn g7c_cylinder_naive_union_on_synth_shell_fires_branch_b_and_c_same_as_hull_pts() {
    let host = build_synthetic_shell_host_manifold(PROD_CELL_SIZE_M);
    let baseline_mesh = manifold_to_indexed_mesh(&host);
    let baseline_components = find_connected_components(&baseline_mesh).component_count;
    let baseline_critical = critical_issue_types(&baseline_mesh);
    assert_eq!(
        baseline_components, 1,
        "synthetic half-shell baseline regression — should be 1 component per §G-7",
    );

    let cyl = Manifold::cylinder(
        2.0 * CYL_HALF_LENGTH_MM,
        CYL_RADIUS_MM,
        CYL_RADIUS_MM,
        CYL_SEGMENTS,
        true,
    );
    let positioned = cyl.translate(PIN_RADIUS_OFFSET_MM, 0.0, PIN_Z_EQUATOR_MM);
    let result = host.union(&positioned);
    let result_mesh = manifold_to_indexed_mesh(&result);
    let post_components = find_connected_components(&result_mesh).component_count;
    let post_critical = critical_issue_types(&result_mesh);
    let new_critical: Vec<_> = post_critical
        .iter()
        .filter(|t| !baseline_critical.contains(t))
        .copied()
        .collect();

    eprintln!(
        "g7c CYLINDER naive union on synth shell:\n  \
         baseline: {} comp / {:?}\n  \
         post:     {} comp / {:?}\n  \
         new crit: {:?}",
        baseline_components, baseline_critical, post_components, post_critical, new_critical,
    );

    // Lock the observed BRANCH B+C behaviour. If this ever flips to
    // PASS (cylinder cleanly unions on the synthetic shell), then a
    // manifold3d upstream fix has shifted the naive-union semantics
    // and the §G-7 framework conclusion needs re-evaluation. The
    // current expectation is BRANCH B+C — same symptoms as the
    // §G-7 hull_pts probe.
    assert!(
        post_components > baseline_components,
        "cylinder mesh-CSG expected to fire BRANCH B (component++) on synth shell — \
         if it doesn't, the §G-7 framework's hull_pts-pyramid-specific reading needs \
         re-evaluation. baseline={baseline_components}, post={post_components}",
    );
    assert!(
        new_critical.contains(&PrintIssueType::SelfIntersecting),
        "cylinder mesh-CSG expected to fire BRANCH C (SelfIntersecting added) on synth shell. \
         new_critical={new_critical:?}",
    );
}
