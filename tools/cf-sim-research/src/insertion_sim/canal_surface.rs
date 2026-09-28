//! Fit plan U17 (soft-contact recon §15g step 6): the product wall's canal surface, measured locally on
//! `base_mold`. The old path's wall puts its canal nodes off the true canal surface (§16r). This finds what
//! does by changing one thing at a time, and what each change costs the stable step.
//!
//! ⛔ The scan never enters the repo. LOCAL lines stay on the machine that ran it; PUBLIC lines are the ratios
//! and verdicts the plan records (Jon, 2026-09-26).

#![cfg(test)]
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]

use std::sync::Arc;
use std::time::Instant;

use cf_cap_planes::{CapPlane, dome_wall_only_mesh};
use cf_design::Solid;
use cf_device_types::SimDesign;
use mesh_repair::weld_vertices;
use mesh_sdf::{ParitySign, PseudoNormalSign, Sign, Signed, TriMeshDistance};
use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};
use sim_soft::lowering::{Lowering, lower};
use sim_soft::{CutPoints, Mesh, Sdf, SdfMeshedTetMesh, TetId, Yeoh};
use sim_soft_explicit::ExplicitModel;
use sim_soft_explicit::executor::Obstacle;
use sim_soft_explicit::fixtures::tube::ECOFLEX_00_30_VISCOUS_TIME;

use super::explicit_budget::{
    Bias, CANAL_WINDOW, D4_SECONDS, PRODUCT_POISSON, STEP_ACCURACY_BAR, Truth, VISCOSITY_SCALES,
    canal_nodes, canal_truth, cost_line, element_size, h_k2, k1_budget, product_loading, quantile,
    scan_grid, scan_obstacle, start_pose, step_error, wall_at_size,
};
use super::{
    Aabb, GRID_SDF_SMOOTH_SIGMA_CELLS, GridSdf, SdfGrid, decimate_for_sdf,
    effective_silicone_for_layer, layer_boundary_thresholds, mesh_wall, scan_aabb, wall_body,
    wall_material_field,
};

/// A signed distance the wall is meshed from.
pub(super) type Field = Arc<dyn cf_design::Sdf>;

/// The old path's SDF source: the scan decimated to this many faces.
const OLD_SDF_FACES: usize = 2_500;

/// The old path's grid spacing over its lattice spacing.
const OLD_GRID_OVER_LATTICE: f64 = 0.75;

/// The box the wall is meshed over, as `build_insertion_geometry` sets it.
fn wall_bounds(scan: &IndexedMesh, design: &SimDesign, cell: f64) -> Aabb {
    let total: f64 = design.layers.iter().map(|l| l.thickness_m).sum();
    scan_aabb(scan, (total - design.cavity_inset_m).max(0.0) + cell)
}

/// `mesh`'s distance on the old path's grid (`build_grid_sdf`): flood-filled at spacing `cell` over `bounds`
/// and pre-smoothed by `sigma_cells`.
fn grid_field(mesh: &IndexedMesh, bounds: Aabb, cell: f64, sigma_cells: f64) -> Field {
    let distance = TriMeshDistance::new(mesh.clone()).expect("the distance must build");
    let (layout, values, _) = scan_grid(&distance, bounds, cell, sigma_cells);
    let side = |n: u32| n as usize;
    Arc::new(GridSdf {
        grid: SdfGrid::new(
            values,
            side(layout.size_x),
            side(layout.size_y),
            side(layout.size_z),
            cell,
            Point3::new(layout.origin_x, layout.origin_y, layout.origin_z),
        ),
    })
}

/// `surface`'s exact distance, negative inside `closed` by parity. Of an open surface's, the wall reads only
/// the magnitude.
fn exact_field(surface: &IndexedMesh, closed: &IndexedMesh) -> Field {
    Arc::new(Signed {
        distance: TriMeshDistance::new(surface.clone()).expect("the distance must build"),
        sign: ParitySign::new(closed).expect("the scan must bin"),
    })
}

/// The product's wall as `build_insertion_geometry` builds it, meshed from `closed` and `open` on a lattice
/// of spacing `cell` with its cut points where `cut_points` puts them: its body and its mesh.
pub(super) fn wall(
    (closed, open): (Field, Field),
    scan: &IndexedMesh,
    design: &SimDesign,
    caps: &[CapPlane],
    cell: f64,
    cut_points: CutPoints,
) -> (Solid, SdfMeshedTetMesh<Yeoh>) {
    let total: f64 = design.layers.iter().map(|l| l.thickness_m).sum();
    let bounds = wall_bounds(scan, design, cell);
    let materials: Vec<_> = design
        .layers
        .iter()
        .map(|layer| effective_silicone_for_layer(layer).unwrap().0)
        .collect();
    let field = wall_material_field(&closed, &layer_boundary_thresholds(design), &materials);
    let planes: Vec<(Point3<f64>, Vector3<f64>)> = caps.iter().map(CapPlane::as_tuple).collect();
    let body = wall_body(
        closed,
        open,
        bounds,
        &planes,
        -design.cavity_inset_m,
        total - design.cavity_inset_m,
    );
    let mesh = mesh_wall(&body, bounds, cell, field, cut_points).expect("the wall must mesh");
    (body, mesh)
}

/// The old path's fields at `design`'s bounds, with the grid pre-smoothed by `sigma_cells`, from `scan`
/// decimated to `faces` faces (`None`: the scan as loaded).
fn old_fields(
    scan: &IndexedMesh,
    design: &SimDesign,
    caps: &[CapPlane],
    cell: f64,
    faces: Option<usize>,
    sigma_cells: f64,
) -> (Field, Field) {
    let source = faces.map_or_else(|| scan.clone(), |faces| decimate_for_sdf(scan, faces));
    let bounds = wall_bounds(scan, design, cell);
    let grid_cell = OLD_GRID_OVER_LATTICE * cell;
    (
        grid_field(&source, bounds, grid_cell, sigma_cells),
        grid_field(
            &dome_wall_only_mesh(&source, caps),
            bounds,
            grid_cell,
            sigma_cells,
        ),
    )
}

/// The scan's exact distances: the closed scan's and the cap-stripped scan's, both signed by the closed
/// scan's parity.
pub(super) fn exact_fields(scan: &IndexedMesh, caps: &[CapPlane]) -> (Field, Field) {
    (
        exact_field(scan, scan),
        exact_field(&dome_wall_only_mesh(scan, caps), scan),
    )
}

/// The p5, p95, mean and largest magnitude of `offsets`.
fn spread(mut offsets: Vec<f64>) -> String {
    offsets.sort_by(f64::total_cmp);
    let mean = offsets.iter().sum::<f64>() / offsets.len() as f64;
    let worst = offsets.iter().fold(0.0, |w: f64, &o| w.max(o.abs()));
    format!(
        "p5 {:+.3} p95 {:+.3} mean {mean:+.3} worst {worst:.3}",
        quantile(&offsets, 0.05),
        quantile(&offsets, 0.95)
    )
}

/// `surface`'s exact distance sampled on the old path's grid (the layout `grid_field` builds, trilinear
/// between samples), negative inside `closed` by parity: that grid with parity's sign in place of the flood
/// fill's.
fn parity_grid_field(
    surface: &IndexedMesh,
    closed: &IndexedMesh,
    bounds: Aabb,
    cell: f64,
) -> Field {
    let distance = TriMeshDistance::new(surface.clone()).expect("the distance must build");
    let (layout, _, _) = scan_grid(&distance, bounds, cell, 0.0);
    let exact = exact_field(surface, closed);
    let side = |n: u32| n as usize;
    let (w, h, d) = (
        side(layout.size_x),
        side(layout.size_y),
        side(layout.size_z),
    );
    let mut values = Vec::with_capacity(w * h * d);
    for k in 0..d {
        for j in 0..h {
            for i in 0..w {
                values.push(exact.eval(Point3::new(
                    layout.origin_x + i as f64 * cell,
                    layout.origin_y + j as f64 * cell,
                    layout.origin_z + k as f64 * cell,
                )));
            }
        }
    }
    Arc::new(GridSdf {
        grid: SdfGrid::new(
            values,
            w,
            h,
            d,
            cell,
            Point3::new(layout.origin_x, layout.origin_y, layout.origin_z),
        ),
    })
}

/// What a wall costs and where its canal nodes sit.
struct Reading {
    /// The rest step, elastic and with Ecoflex 00-30's viscosity (ν 0.49).
    steps: (f64, f64),
    /// Where the canal nodes sit.
    canal: Vec<Point3<f64>>,
}

/// Everything the budget's cost line needs besides the model.
struct Costing<'a> {
    obstacle: &'a Obstacle,
    loading: (f64, f64),
    k1: (f64, usize),
}

/// Measures a wall built in `meshing` seconds (its fields and its mesh): its canal nodes against `truth` at `inset`, split into how
/// far each sits off the mesher's own zero set (`-body` at the node) and the rest, the field's error at the
/// node; and its rest step, elastic and viscous at ν 0.49 and at the budget's worst corner (ν 0.495, the
/// viscosity's top), printed with a press over D4 (`costing`). With `accuracy`, also §16e's step accuracy at
/// ν 0.49.
///
/// The split holds where the cavity's distance is the rind's, `inset − |open|`, on the canal side of the
/// scan's surface: there `-body` is the node's offset from the mesher's zero set, in its field's units. A
/// canal node outside the scan (possible at a small inset) reads the rind's other branch, and its "field's
/// error" is not one. At 0 mm there is no rind, and the split holds at every canal node.
#[allow(clippy::too_many_arguments)] // a probe's report: each argument is a column of it
fn measure(
    label: &str,
    (body, mesh, meshing): (&Solid, &SdfMeshedTetMesh<Yeoh>, f64),
    design: &SimDesign,
    truth: &Truth,
    planes: &[(Point3<f64>, Vector3<f64>)],
    target: f64,
    costing: &Costing<'_>,
    accuracy: bool,
) -> Reading {
    let density = cf_device_types::material_density(&design.layers[0].anchor_key);
    let densities = vec![density; mesh.n_tets()];
    let lowered = |viscous_time: f64, poisson: f64| {
        lower(
            mesh,
            &densities,
            Lowering {
                poisson,
                viscous_time,
            },
            &[],
        )
        .unwrap()
    };
    let (elastic, viscous) = (
        lowered(0.0, 0.49),
        lowered(ECOFLEX_00_30_VISCOUS_TIME, 0.49),
    );
    let worst_corner = lowered(
        VISCOSITY_SCALES[2] * ECOFLEX_00_30_VISCOUS_TIME,
        PRODUCT_POISSON[1],
    );
    let size = element_size(&elastic.model);
    let inset = design.cavity_inset_m;
    let (canal, beyond) = canal_nodes(&elastic.model, truth, planes, inset, size);
    let at = |model: &ExplicitModel, n: u32| {
        let [x, y, z] = model.rest_positions()[n as usize];
        Point3::new(x, y, z)
    };
    let points: Vec<Point3<f64>> = canal.iter().map(|&n| at(&elastic.model, n)).collect();
    let totals: Vec<f64> = points
        .iter()
        .map(|&p| (truth.signed(p) + inset) / size)
        .collect();
    let past = totals.iter().filter(|o| o.abs() > 0.1).count();
    let worst = points
        .iter()
        .zip(&totals)
        .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
        .map(|(&p, _)| p)
        .unwrap();
    let from_the_caps = planes
        .iter()
        .map(|(centroid, normal)| (worst - centroid).dot(&normal.normalize()).abs())
        .fold(f64::INFINITY, f64::min)
        / size;
    let (own, field): (Vec<f64>, Vec<f64>) = points
        .iter()
        .map(|&p| {
            let total = truth.signed(p) + inset;
            let own = -body.eval(p);
            (own / size, (total - own) / size)
        })
        .unzip();
    let outside_the_scan = if inset > 0.0 {
        let count = points.iter().filter(|&&p| !truth.sign.is_inside(p)).count();
        format!("{count}")
    } else {
        "none, no rind at 0 mm".to_owned()
    };
    println!(
        "{label}: h/h_K2 {:.3} [PUBLIC]; {} elements, built in {meshing:.2} s, {:.4} of D4 [LOCAL]",
        size / target,
        elastic.model.element_count(),
        meshing / D4_SECONDS
    );
    println!(
        "  against the truth: {} | boundary nodes {CANAL_WINDOW}–{} h from the level {beyond} [PUBLIC but the counts]",
        Bias::of(&elastic.model, &canal, truth, inset, size).line(),
        CANAL_WINDOW + 1.0
    );
    println!(
        "  past 0.1 h: {past}; where the split does not hold (outside the scan, past the rind's kink): {outside_the_scan} [PUBLIC but the counts]; the worst {from_the_caps:.2} h from a cap plane [LOCAL]"
    );
    println!("  off the mesher's own level, /h: {} [PUBLIC]", spread(own));
    println!(
        "  the field's error at the node, /h: {} [PUBLIC]",
        spread(field)
    );
    let step = |label: &str, model: &ExplicitModel| {
        cost_line(
            label,
            model,
            costing.obstacle,
            costing.loading,
            costing.k1,
            false,
        )
    };
    let steps = (
        step("  ν 0.49, η off", &elastic.model),
        step("  ν 0.49, η on", &viscous.model),
    );
    step(
        "  ν 0.495, η ×1.471 (the budget's worst corner)",
        &worst_corner.model,
    );
    if accuracy {
        for (label, model) in [("η off", &elastic.model), ("η on", &viscous.model)] {
            let (error, drift) = step_error(model, costing.obstacle);
            println!(
                "  step accuracy, ν 0.49, {label}: {error:+.4} (bar ±{STEP_ACCURACY_BAR}; drift {drift:+.1e}) [PUBLIC]"
            );
        }
    }
    Reading {
        steps,
        canal: points,
    }
}

/// U17's causes on `base_mold`, at the 5 mm inset and at 1 mm and 0 mm (D3's search reaches 0 mm): the old
/// path's wall at h_K2, then one change at a time, each from the one before: no pre-smooth; the scan as loaded,
/// not decimated; the grid signed by parity, not the flood fill; the scan's exact distance for the grid; the cut
/// points located on that distance, not interpolated. Each against the parity-signed canal truth, split into the
/// mesher's own placement and the field's error, with the truth's sign against a 1 mm flood fill and the
/// welded scan's pseudo-normals. Prints a report; asserts only that it rebuilt the old path's wall.
///
/// `RAYON_NUM_THREADS=4 cargo test --release -p cf-sim-research --bin cf-sim-research --
/// insertion_sim::canal_surface --ignored --nocapture`, with the scan at `~/scans/base_mold.cleaned.stl` (or
/// `CF_SIM_RESEARCH_PRODUCT_SCAN`).
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn where_the_canal_nodes_offsets_come_from_on_the_product_scan() {
    let (scan, centerline, caps, design) = super::tests::product_scene()
        .expect("U17 measures the product scan, which must be present");
    let planes: Vec<(Point3<f64>, Vector3<f64>)> = caps.iter().map(CapPlane::as_tuple).collect();
    println!("\n══ U17: where the canal nodes' offsets come from ══");
    let (target, _) = h_k2();
    let (geometry, cell) = wall_at_size(&scan, &design, &caps, target, OLD_SDF_FACES);
    println!(
        "lattice {:.3} mm, scan {} faces [LOCAL]",
        1e3 * cell,
        scan.faces.len()
    );

    let closed = TriMeshDistance::new(scan.clone()).expect("the scan's distance must build");
    let (grid_1mm, values_1mm, flood) = scan_grid(&closed, scan_aabb(&scan, 0.004), 0.001, 0.0);
    let obstacle = scan_obstacle(grid_1mm, values_1mm, start_pose(&centerline));
    let costing = Costing {
        obstacle: &obstacle,
        loading: product_loading(&geometry, &design, &centerline),
        k1: k1_budget(),
    };
    let truth = canal_truth(&scan, &caps);
    let mut welded = scan.clone();
    weld_vertices(&mut welded, 1e-6);
    let pseudo = PseudoNormalSign::from_distance(
        &TriMeshDistance::new(welded).expect("the welded scan's distance must build"),
    );
    let sides = dome_wall_only_mesh(&scan, &caps);

    for inset in [design.cavity_inset_m, 0.001, 0.0] {
        let at_inset = SimDesign {
            cavity_inset_m: inset,
            layers: design.layers.clone(),
        };
        println!("── inset {:.1} mm ──", 1e3 * inset);
        let bounds = wall_bounds(&scan, &at_inset, cell);
        let grid_cell = OLD_GRID_OVER_LATTICE * cell;
        // One change at a time, each from the one before.
        let chain: [(&str, Box<dyn Fn() -> (Field, Field)>, CutPoints); 6] = [
            (
                "old path (decimated, grid, pre-smoothed)",
                Box::new(|| {
                    old_fields(
                        &scan,
                        &at_inset,
                        &caps,
                        cell,
                        Some(OLD_SDF_FACES),
                        GRID_SDF_SMOOTH_SIGMA_CELLS,
                    )
                }),
                CutPoints::Interpolated,
            ),
            (
                "no pre-smooth",
                Box::new(|| old_fields(&scan, &at_inset, &caps, cell, Some(OLD_SDF_FACES), 0.0)),
                CutPoints::Interpolated,
            ),
            (
                "no pre-smooth, the scan as loaded",
                Box::new(|| old_fields(&scan, &at_inset, &caps, cell, None, 0.0)),
                CutPoints::Interpolated,
            ),
            (
                "the grid signed by parity",
                Box::new(|| {
                    (
                        parity_grid_field(&scan, &scan, bounds, grid_cell),
                        parity_grid_field(&sides, &scan, bounds, grid_cell),
                    )
                }),
                CutPoints::Interpolated,
            ),
            (
                "the exact distance",
                Box::new(|| exact_fields(&scan, &caps)),
                CutPoints::Interpolated,
            ),
            (
                "the exact distance, cuts located on it",
                Box::new(|| exact_fields(&scan, &caps)),
                CutPoints::Root,
            ),
        ];
        let mut old: Option<Reading> = None;
        for (index, (label, fields, cut_points)) in chain.iter().enumerate() {
            let started = Instant::now();
            let (body, mesh) = wall(fields(), &scan, &at_inset, &caps, cell, *cut_points);
            let meshing = started.elapsed().as_secs_f64();
            if index == 0 && inset == design.cavity_inset_m {
                // It must be the wall `build_insertion_geometry` built.
                let tets = |mesh: &SdfMeshedTetMesh<Yeoh>| {
                    (0..mesh.n_tets())
                        .map(|t| mesh.tet_vertices(TetId::try_from(t).unwrap()))
                        .collect::<Vec<_>>()
                };
                assert_eq!(
                    (mesh.positions(), tets(&mesh)),
                    (geometry.mesh.positions(), tets(&geometry.mesh)),
                    "the probe must rebuild the old path's wall"
                );
            }
            let reading = measure(
                label,
                (&body, &mesh, meshing),
                &at_inset,
                &truth,
                &planes,
                target,
                &costing,
                *cut_points == CutPoints::Root,
            );
            let pseudo_disagrees = reading
                .canal
                .iter()
                .filter(|&&p| truth.sign.is_inside(p) != pseudo.is_inside(p))
                .count();
            let flood_disagrees = reading
                .canal
                .iter()
                .filter(|&&p| truth.sign.is_inside(p) != (flood.signed_distance(p) < 0.0))
                .count();
            println!(
                "  canal nodes where parity disagrees with the welded pseudo-normals {pseudo_disagrees}, with a 1 mm flood fill {flood_disagrees} [PUBLIC but the counts]"
            );
            match &old {
                Some(baseline) => print_steps(label, &reading, baseline),
                None => old = Some(reading),
            }
        }
    }
}

/// A wall's rest steps over `baseline`'s.
fn print_steps(label: &str, reading: &Reading, baseline: &Reading) {
    println!(
        "  {label}: the step over the old path's, η off {:.4}, η on {:.4} [PUBLIC]",
        reading.steps.0 / baseline.steps.0,
        reading.steps.1 / baseline.steps.1
    );
}
