//! Step 6's lowering on the product scan (soft-contact recon §16w): step 7's wall at h_K2, lowered and mounted at its
//! closed end; the scan's fitted path, its start and its sampling in time; the bake; and a frictionless run from the
//! start through the hold. Beside them: whether the wall is one piece, whether its surface touches itself, how many
//! vertices inside it the old pin's rule would take, and a diagnostic of §16r's non-finite attempt.
//!
//! ⛔ The scan never enters the repo. LOCAL lines stay on the machine that ran it; PUBLIC lines are the ratios and
//! verdicts the plan records (Jon, 2026-09-26).

#![cfg(test)]
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]

use std::collections::HashMap;
use std::f64::consts::TAU;
use std::sync::Arc;
use std::time::Instant;

use cf_device_types::SimDesign;
use nalgebra::{Point3, Vector3};
use sim_soft::lowering::path::{Centreline, FittedPath};
use sim_soft::lowering::{
    Lowered, Lowering, PRODUCT_BAKE, Plane, SAMPLING_BAR, START_CLEARANCE, Skin, lower,
};
use sim_soft::obstacle::{SignedDistance, bake_surface};
use sim_soft::{CutPoints, Mesh, Sdf, SdfMeshedTetMesh, TetId, VertexId, Yeoh};
use sim_soft_explicit::ExplicitModel;
use sim_soft_explicit::cpu::f64::CpuExecutor;
use sim_soft_explicit::executor::{Executor, Monitors, Obstacle};
use sim_soft_explicit::fixtures::tube::ECOFLEX_00_30_VISCOUS_TIME;
use sim_soft_explicit::stepping::{Stepper, StepperConfig};

use super::canal_surface::{exact_fields, wall};
use super::explicit_budget::{
    D4_SECONDS, element_size, g2_bar, h_k2, rest_step, speed_over_shear_wave, start_pose,
};

/// The run's Poisson's ratio: the product's lower one (§16r).
pub(super) const POISSON: f64 = 0.49;

/// §15b's hold after the loading.
pub(super) const HOLD: f64 = 0.2;

/// A field less a constant: the scan's signed distance less the outer offset is the outer surface's own level.
pub(super) struct Less {
    pub(super) field: Arc<dyn cf_design::Sdf>,
    pub(super) by: f64,
}

impl Sdf for Less {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.field.eval(p) - self.by
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        self.field.grad(p)
    }
}

/// Step 7's wall at `cell`: the scan's exact distances, its cut points located on them.
pub(super) fn step7_wall(
    scan: &mesh_types::IndexedMesh,
    design: &SimDesign,
    caps: &[cf_cap_planes::CapPlane],
    cell: f64,
) -> SdfMeshedTetMesh<Yeoh> {
    wall(
        exact_fields(scan, caps),
        scan,
        design,
        caps,
        cell,
        CutPoints::Root,
    )
    .1
}

/// Each element's density: the one layer's.
pub(super) fn densities(design: &SimDesign, mesh: &SdfMeshedTetMesh<Yeoh>) -> Vec<f64> {
    assert_eq!(design.layers.len(), 1, "the product has one layer");
    vec![cf_device_types::material_density(&design.layers[0].anchor_key); mesh.n_tets()]
}

/// Step 7's wall near `target` element size: a first mesh at the old path's 4 mm lattice, then secant steps on the
/// lattice spacing until the element size is within 2 % of the target (at most three).
pub(super) fn step7_wall_at_size(
    scan: &mesh_types::IndexedMesh,
    design: &SimDesign,
    caps: &[cf_cap_planes::CapPlane],
    target: f64,
) -> (SdfMeshedTetMesh<Yeoh>, f64) {
    let mut cell = 0.004;
    for attempt in 0..4 {
        let mesh = step7_wall(scan, design, caps, cell);
        let lowered = lower(
            &mesh,
            &densities(design, &mesh),
            Lowering {
                poisson: POISSON,
                viscous_time: 0.0,
            },
            &[],
        )
        .unwrap();
        let size = element_size(&lowered.model);
        println!(
            "  mesh {attempt}: lattice {:.3} mm, h {:.3} mm ({:+.1} % of the target) [LOCAL]",
            1e3 * cell,
            1e3 * size,
            100.0 * (size / target - 1.0)
        );
        if (size / target - 1.0).abs() <= 0.02 || attempt == 3 {
            return (mesh, cell);
        }
        cell *= target / size;
    }
    unreachable!("the loop returns on its last attempt")
}

/// The root of `a`'s set in a union–find forest, halving the path on the way.
fn root(parent: &mut [usize], mut a: usize) -> usize {
    while parent[a] != a {
        parent[a] = parent[parent[a]];
        a = parent[a];
    }
    a
}

/// How many pieces a model's elements make, joined through shared nodes.
pub(super) fn pieces(model: &ExplicitModel) -> usize {
    let mut parent: Vec<usize> = (0..model.node_count()).collect();
    for element in model.elements() {
        let first = root(&mut parent, element[0] as usize);
        for &n in &element[1..] {
            let other = root(&mut parent, n as usize);
            parent[other] = first;
        }
    }
    (0..model.node_count())
        .filter(|&a| root(&mut parent, a) == a)
        .count()
}

/// Where a model's surface touches itself: its boundary edges not on exactly two boundary triangles, and its
/// boundary nodes whose triangles form more than one fan (triangles around a node joined through the edges they
/// share there).
fn touches(model: &ExplicitModel) -> (usize, usize) {
    let triangles = model.surface_triangles();
    let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
    let mut around: HashMap<u32, Vec<usize>> = HashMap::new();
    for (t, triangle) in triangles.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (triangle[k], triangle[(k + 1) % 3]);
            *edges.entry((a.min(b), a.max(b))).or_insert(0) += 1;
            around.entry(triangle[k]).or_default().push(t);
        }
    }
    let bad_edges = edges.values().filter(|&&count| count != 2).count();
    let fans = |node: u32, faces: &[usize]| {
        // Triangles at `node` joined when they share an edge from it.
        let mut parent: Vec<usize> = (0..faces.len()).collect();
        let others = |t: usize| {
            let triangle = triangles[faces[t]];
            triangle
                .into_iter()
                .filter(|&v| v != node)
                .collect::<Vec<_>>()
        };
        for i in 0..faces.len() {
            for j in (i + 1)..faces.len() {
                if others(i).iter().any(|v| others(j).contains(v)) {
                    let (a, b) = (root(&mut parent, i), root(&mut parent, j));
                    parent[a] = b;
                }
            }
        }
        (0..faces.len())
            .filter(|&i| root(&mut parent, i) == i)
            .count()
    };
    let bad_nodes = around
        .iter()
        .filter(|(node, faces)| fans(**node, faces) > 1)
        .count();
    (bad_edges, bad_nodes)
}

/// The model's boundary nodes' rest positions.
pub(super) fn boundary_points(model: &ExplicitModel) -> Vec<Point3<f64>> {
    let mut nodes: Vec<u32> = model
        .surface_triangles()
        .iter()
        .flatten()
        .copied()
        .collect();
    nodes.sort_unstable();
    nodes.dedup();
    nodes
        .into_iter()
        .map(|n| Point3::from(model.rest_positions()[n as usize]))
        .collect()
}

/// A run's last monitors and whether it stopped early, at which step.
fn run(
    model: &ExplicitModel,
    obstacle: &Obstacle,
    damping: f64,
    end: f64,
) -> (Monitors, Option<u64>, u64) {
    let executor = CpuExecutor::new(model, obstacle).unwrap();
    let mut stepper = Stepper::new(executor, StepperConfig::new(damping), 0.0);
    let stopped = stepper.run_until(end).err().map(|_| stepper.steps());
    let steps = stepper.steps();
    (stepper.executor_mut().monitors(), stopped, steps)
}

/// 100 steps of `executor` from rest with no mass damping: the step that failed, if one did, and the monitors.
fn held_still<E: Executor>(executor: E) -> (Option<u64>, Monitors) {
    let mut stepper = Stepper::new(executor, StepperConfig::new(0.0), 0.0);
    let mut first_bad = None;
    for step in 0..100 {
        if stepper.step().is_err() {
            first_bad = Some(step);
            break;
        }
    }
    (first_bad, stepper.executor_mut().monitors())
}

/// §16w's product checks. Prints a report; asserts only that each piece built.
///
/// `RAYON_NUM_THREADS=4 cargo test --release -p cf-sim-research --bin cf-sim-research --
/// insertion_sim::product_lowering --ignored --nocapture`, with the scan at `~/scans/base_mold.cleaned.stl` (or
/// `CF_SIM_RESEARCH_PRODUCT_SCAN`).
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn the_lowering_on_the_product_scan() {
    let (scan, centerline, caps, design) = super::tests::product_scene()
        .expect("the lowering is checked on the product scan, which must be present");
    println!("\n══ step 6's lowering on base_mold ══");
    let started = Instant::now();
    let (target, _) = h_k2();
    let (mesh, cell) = step7_wall_at_size(&scan, &design, &caps, target);
    let densities = densities(&design, &mesh);
    let (closed, _) = exact_fields(&scan, &caps);
    let total: f64 = design.layers.iter().map(|l| l.thickness_m).sum();
    let outer = Less {
        field: closed,
        by: total - design.cavity_inset_m,
    };
    let skin = Skin::of(&mesh, &outer, cell);
    let seated_tip = Centreline::new(&centerline).unwrap().tangent(0.0);
    let mount = skin.beyond(&mesh, &Plane::new(centerline[0], -seated_tip).unwrap());
    assert!(!mount.is_empty(), "the mount must hold the wall");
    let lowering = Lowering {
        poisson: POISSON,
        viscous_time: ECOFLEX_00_30_VISCOUS_TIME,
    };
    let Lowered { model, .. } = lower(&mesh, &densities, lowering, &mount).unwrap();
    println!(
        "wall: {} elements, {} nodes, skin {} vertices, mount {} [LOCAL]; built and lowered in {:.1} s [LOCAL]",
        model.element_count(),
        model.node_count(),
        skin.vertices().len(),
        mount.len(),
        started.elapsed().as_secs_f64()
    );
    println!(
        "wall: h / h_K2 {:.3}; one piece: {} [PUBLIC]",
        element_size(&model) / target,
        pieces(&model) == 1
    );

    // The old pin's rule: every vertex the elements name within half a cell of the outer surface.
    let mut boundary: Vec<VertexId> = mesh.boundary_faces().iter().flatten().copied().collect();
    boundary.sort_unstable();
    boundary.dedup();
    let mut named: Vec<VertexId> = (0..mesh.n_tets())
        .flat_map(|t| mesh.tet_vertices(TetId::try_from(t).unwrap()))
        .collect();
    named.sort_unstable();
    named.dedup();
    let near = |v: &VertexId| {
        outer
            .eval(Point3::from(mesh.positions()[*v as usize]))
            .abs()
            < 0.5 * cell
    };
    let old_rule = named.iter().filter(|v| near(v)).count();
    let inside_the_wall = named
        .iter()
        .filter(|v| near(v) && boundary.binary_search(v).is_err())
        .count();
    let skin_inside = skin
        .vertices()
        .iter()
        .filter(|v| boundary.binary_search(v).is_err())
        .count();
    println!(
        "old pin's rule: {old_rule} vertices, {inside_the_wall} of them inside the wall [LOCAL]; the old rule takes \
         some inside: {} [PUBLIC]; skin vertices inside the wall: {skin_inside} [PUBLIC]",
        inside_the_wall > 0
    );
    let (edges, nodes) = touches(&model);
    println!(
        "surface: {edges} edges not on two triangles, {nodes} nodes with more than one fan [LOCAL]; touches itself: {} [PUBLIC]",
        edges + nodes > 0
    );

    // The path.
    let device: Vec<Plane> = caps
        .iter()
        .map(|cap| Plane::new(cap.centroid, cap.normal).unwrap())
        .collect();
    let started = Instant::now();
    let path = FittedPath::new(&scan, Centreline::new(&centerline).unwrap(), device).unwrap();
    let length = path.centreline().length();
    let join = path.join().expect("the scan leaves the device");
    println!(
        "path: built in {:.1} s; join {:.2} mm of a {:.1} mm centreline [LOCAL]; join / length {:.2} [PUBLIC]",
        started.elapsed().as_secs_f64(),
        1e3 * join,
        1e3 * length,
        join / length
    );
    let at_join = path.fitted(join).unwrap().rotation;
    for back in [0.02, 0.01, 0.005, 0.002, 0.001, 0.0005] {
        let turn = path
            .fitted(join - back)
            .unwrap()
            .rotation
            .angle_to(&at_join);
        println!(
            "  the fitted rotation {:.1} mm before the join is {:.4}° from the join's [PUBLIC]",
            1e3 * back,
            turn.to_degrees()
        );
    }
    let started = Instant::now();
    let scan_distance = SignedDistance::new(&scan).unwrap();
    let start = path
        .start(&boundary_points(&model), &scan_distance, START_CLEARANCE)
        .unwrap();
    println!(
        "start: walk {:.2} mm, found in {:.1} s [LOCAL]; at the join: {}; longer than the centreline and §15b's 5 mm: \
         {} [PUBLIC]",
        1e3 * start,
        started.elapsed().as_secs_f64(),
        start == join,
        start > length + 0.005
    );
    let innermost = mesh.materials()[0].mu();
    let speed = speed_over_shear_wave() * (innermost / densities[0]).sqrt();
    let loading = start / (0.9 * speed);
    let started = Instant::now();
    let sampled = path.sampled(start, loading, SAMPLING_BAR).unwrap();
    println!(
        "sampling: {} intervals, in {:.1} s [LOCAL]; loading {loading:.4} s [LOCAL]",
        sampled.poses.len() - 1,
        started.elapsed().as_secs_f64()
    );
    for (intervals, error) in &sampled.errors {
        println!(
            "  {intervals} intervals: the interpolated pose strays {:.3} of the bar [PUBLIC]",
            error / SAMPLING_BAR
        );
    }

    // The bake.
    let started = Instant::now();
    let baked = bake_surface(&scan_distance, PRODUCT_BAKE).unwrap();
    let bake_seconds = started.elapsed().as_secs_f64();
    println!(
        "bake: {:.1} s, {} coarse values, {} fine values ({:.0} MB at f64) [LOCAL]; band {:.2} mm, {} fine cells [PUBLIC]",
        bake_seconds,
        baked.values.len(),
        baked.fine.values.len(),
        baked.fine.values.len() as f64 * 8e-6,
        1e3 * PRODUCT_BAKE.band,
        (PRODUCT_BAKE.band / PRODUCT_BAKE.fine_cell).round()
    );
    println!(
        "bake: {:.2} of D4, once per scan; the fine values at f32 over wgpu's default storage binding (128 MiB) {:.1} \
         [LOCAL]; the record keeps rough bounds",
        bake_seconds / D4_SECONDS,
        baked.fine.values.len() as f64 * 4.0 / (128.0 * 1024.0 * 1024.0)
    );

    // A frictionless run from the start through the hold, damped as the tube is (ξ 0.05 at the shear wave's period
    // along the centreline).
    let obstacle = sampled.obstacle(baked, 0.0);
    let shear_period = 4.0 * length / (innermost / densities[0]).sqrt();
    let damping = 2.0 * 0.05 * (TAU / shear_period);
    let started = Instant::now();
    let (monitors, stopped, steps) = run(&model, &obstacle, damping, loading + HOLD);
    println!(
        "run: {steps} steps in {:.1} s [LOCAL]; stopped early: {stopped:?}; finite {} inverted element-steps {} [PUBLIC]",
        started.elapsed().as_secs_f64(),
        monitors.finite(),
        monitors.inverted_element_steps
    );
    let band = PRODUCT_BAKE.band;
    println!(
        "  deepest predicted point {:.3} of the band, {:.2} fine cells; corrections from the coarse grid {}; deepest \
         penetration on the grid {:.3} of G2's bar [PUBLIC]",
        monitors.deepest_prediction / band,
        monitors.deepest_prediction / PRODUCT_BAKE.fine_cell,
        monitors.coarse_corrections,
        monitors.max_penetration / g2_bar(design.cavity_inset_m)
    );
    if monitors.deepest_prediction > 0.5 * band {
        println!(
            "  ⇒ the band rule (§16w): the band becomes {:.0} fine cells",
            (2.0 * monitors.deepest_prediction / PRODUCT_BAKE.fine_cell).ceil()
        );
    }

    // The rest step, mounted over unheld: held nodes take no part in it.
    let unheld = |viscous_time: f64| {
        lower(
            &mesh,
            &densities,
            Lowering {
                poisson: POISSON,
                viscous_time,
            },
            &[],
        )
        .unwrap()
        .model
    };
    let viscous_unheld = unheld(ECOFLEX_00_30_VISCOUS_TIME);
    println!(
        "rest step, mounted over unheld (viscous, ν 0.49): {:.4} [PUBLIC]",
        rest_step(&model, &obstacle) / rest_step(&viscous_unheld, &obstacle)
    );

    // The diagnostic: the scan held still at the old path's start pose for 100 steps with no mass damping. §16r's
    // timed runs held no node and ran on the f32 executor, one elastic and one viscous; the mounted rows change the
    // hold, and the last the precision too.
    let held = Obstacle {
        start: 0.0,
        interval: 1.0,
        poses: vec![start_pose(&centerline)],
        ..obstacle
    };
    let elastic_unheld = unheld(0.0);
    let cases: [(&str, &ExplicitModel, bool); 4] = [
        ("unheld, elastic, f32 (as §16r)", &elastic_unheld, true),
        ("unheld, viscous, f32 (as §16r)", &viscous_unheld, true),
        ("mounted, viscous, f32", &model, true),
        ("mounted, viscous, f64", &model, false),
    ];
    for (label, diagnosed, narrow) in cases {
        let (first_bad, diagnostic) = if narrow {
            held_still(sim_soft_explicit::cpu::f32::CpuExecutor::new(diagnosed, &held).unwrap())
        } else {
            held_still(CpuExecutor::new(diagnosed, &held).unwrap())
        };
        println!(
            "diagnostic, the old start pose held still for 100 steps, {label}: stopped at {first_bad:?}; finite {}; \
             inverted element-steps {} [PUBLIC]; deepest penetration {:.2} mm [LOCAL]",
            diagnostic.finite(),
            diagnostic.inverted_element_steps,
            1e3 * diagnostic.max_penetration
        );
    }
    assert!(steps > 0);
}
