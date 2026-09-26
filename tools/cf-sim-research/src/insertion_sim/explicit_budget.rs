//! Build step 2d (`docs/SOFT_CONTACT_ARCHITECTURE_RECON.md` §15g step 2, §16j): the product's budget on the
//! explicit solver, measured locally on `base_mold`.
//!
//! The wall is meshed with this tool's current Tet4 mesher ([`build_insertion_geometry`]) at the element size
//! K2 needs, lowered into `sim-soft-explicit`'s [`ExplicitModel`] at the Poisson's ratios K2 was judged at, and
//! measured: the stable step with the viscosity off and on and its accuracy, the per-press time against D4's
//! 5 minutes, the surface bias with and without projecting the canal nodes onto the true surface, and the
//! baked scan grid's error against the scan (G2).
//!
//! The lowering here is a measuring copy; lowering moves to `sim-soft` in step 6 (§16j).
//!
//! ⛔ The scan never enters the repo. [`the_products_budget_on_the_explicit_solver`] prints to the terminal,
//! and the plan records only its ratios and verdict (Jon, 2026-09-26): the element count, the step, the step
//! counts and the loading time stay on the machine that ran it.

#![cfg(test)]
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]

use std::time::Instant;

use mesh_sdf::{CachedGridSdf, TriMeshDistance, UnsignedDistance};
use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};
use sim_soft::{Mesh, SdfMeshedTetMesh, TetId, Vec3, VertexId, Yeoh};
use sim_soft_explicit::cpu;
use sim_soft_explicit::executor::{Executor, Obstacle};
use sim_soft_explicit::f64::{Material, Pose, SdfGridLayout};
use sim_soft_explicit::fixtures::golden::THICK_TUBE;
use sim_soft_explicit::fixtures::tube::{
    ECOFLEX_00_30_VISCOUS_TIME, Insertion, Mesh as TubeMesh, Tube, TubeRun, Walls,
};
use sim_soft_explicit::stepping::{Stepper, StepperConfig};
use sim_soft_explicit::{ExplicitModel, ModelError};

use super::{
    Aabb, GRID_SDF_SMOOTH_SIGMA_CELLS, InsertionGeometry, build_insertion_geometry,
    gaussian_smooth_3d_separable, polyline_arc_length_m, scan_aabb, slide_pose_at,
};

/// K2's bar (§15a): the band pressure within 5 % of the oracle.
const K2_BAR: f64 = 0.05;

/// K2 on the tube at 10k and 50k, one row per corner (§16p's table, `T = 10 T_s`; under the kinematic law
/// the raw and gap-corrected errors agree to the printed digits).
const K2_TUBE: [(&str, f64, f64); 4] = [
    ("λ_a 1.1, ν 0.49", 0.0441, 0.0147),
    ("λ_a 1.1, ν 0.495", 0.0529, 0.0160),
    ("λ_a 1.3, ν 0.49", 0.0330, 0.0113),
    ("λ_a 1.3, ν 0.495", 0.0378, 0.0120),
];

/// The product's Poisson's ratios: the two K2 was judged at (§15a). The silicone catalog's own λ is 4μ,
/// ν 0.4 (§5b), which the explicit solver does not use.
const PRODUCT_POISSON: [f64; 2] = [0.49, 0.495];

/// Ecoflex 00-30's viscosity over its η/μ's 7 Pa·s: the published fits' range, 5.2–10.3 Pa·s (fit plan U15).
const VISCOSITY_SCALES: [f64; 3] = [5.2 / 7.0, 1.0, 10.3 / 7.0];

/// The smallest in-run step over the rest step on the damped tube (§16p).
const LOADED_STEP_FACTOR: f64 = 0.977;

/// The loading-time ladder's last valid rung, in shear periods `T_s` (§16p).
const LOADING_RUNG: f64 = 0.625;

/// Runs per verdict: stiffness scaling holds, so 3 (§16p, §15h).
const RUNS_PER_VERDICT: f64 = 3.0;

/// K1 (§15a): a 100k-tet insertion within 2 minutes.
const K1_SECONDS: f64 = 120.0;

/// D4 (§9 decision 12, fit plan U13): 5 minutes per press.
const D4_SECONDS: f64 = 300.0;

/// G2 (fit plan): no node deeper than 1 % of the inset.
const G2_FRACTION_OF_INSET: f64 = 0.01;

/// How far a projected node may leave an element's rest volume: each incident element keeps at least this
/// fraction of it (`SdfMeshedTetMesh::with_projected_nodes`).
const PROJECTION_FLOORS: [f64; 2] = [0.5, 0.1];

/// A canal node within this many element sizes of the true canal surface counts as on it.
const ON_THE_SURFACE: f64 = 0.01;

/// The canal nodes are the boundary nodes within this many element sizes of the true canal surface.
const CANAL_WINDOW: f64 = 2.0;

/// The step's accuracy bar (§16e, in §16p's form): the loop's step within 2 % of 0.9 of the converged
/// critical step.
const STEP_ACCURACY_BAR: f64 = 0.02;

/// Steps the CPU executor is timed over. From a fresh loop they include one of its re-estimates (at step 500;
/// the start's estimates precede the timer), where a run makes one every 500 steps.
const TIMED_STEPS: u64 = 1_000;

/// The value at fraction `q` of `sorted`, nearest rank.
fn quantile(sorted: &[f64], q: f64) -> f64 {
    sorted[(q * (sorted.len() - 1) as f64).round() as usize]
}

/// The stop rule's element size, (mean tet volume)^⅓ (§15g step 2).
fn element_size(model: &ExplicitModel) -> f64 {
    let volume: f64 = model.rest_volumes().iter().sum();
    (volume / model.element_count() as f64).cbrt()
}

/// Where a power law `e = C·h^p` through `(h, e)` at `coarse` and at `fine` reaches `bar`: the stop rule's
/// model, fitted per corner from two meshes (§15g step 2).
fn size_at_bar(coarse: (f64, f64), fine: (f64, f64), bar: f64) -> f64 {
    let power = (coarse.1 / fine.1).ln() / (coarse.0 / fine.0).ln();
    coarse.0 * (bar / coarse.1).powf(power.recip())
}

/// The tube's element size on `mesh` (§15c).
fn tube_size(mesh: TubeMesh) -> f64 {
    let material = tube_run(TubeMesh::TenK).material();
    element_size(&Tube::plan(mesh).model(material, Walls::Free).unwrap())
}

/// `h_K2` (§16j): the largest element size at which K2's error is within its bar at every corner, from the
/// stop rule's model fitted to the 10k and 50k tubes; and the corner that sets it.
fn h_k2() -> (f64, &'static str) {
    let (coarse, fine) = (tube_size(TubeMesh::TenK), tube_size(TubeMesh::FiftyK));
    K2_TUBE
        .iter()
        .map(|&(corner, at_coarse, at_fine)| {
            (
                size_at_bar((coarse, at_coarse), (fine, at_fine), K2_BAR),
                corner,
            )
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .unwrap()
}

/// The tube K1 is set on: plan §15b's case at λ_a 1.1 and ν 0.49, frictionless, Ecoflex 00-30 with its
/// viscosity, loaded at the ladder's rung.
fn tube_run(mesh: TubeMesh) -> TubeRun {
    let (mu, density) = (23.0e3, 1070.0);
    TubeRun {
        mesh,
        case: THICK_TUBE[0],
        mu,
        viscous_time: ECOFLEX_00_30_VISCOUS_TIME,
        density,
        insertion: Insertion::plan(LOADING_RUNG * TubeRun::shear_period(mu, density)),
        window: 0.1,
        friction: 0.0,
        grid_cell: Tube::plan(mesh).inner_radius / 20.0,
    }
}

/// The loading speed over the shear wave speed at the ladder's rung (§16p's v/c_s, 0.39): plan §15b's
/// travel over the insertion's constant-speed share of the loading time, at `c_s = 1`.
fn speed_over_shear_wave() -> f64 {
    let insertion = Insertion::plan(LOADING_RUNG * TubeRun::shear_period(1.0, 1.0));
    (insertion.start_gap + insertion.depth) / (0.9 * insertion.loading_time)
}

/// The steps a run takes: its loading and hold at the rest step times the loaded step factor.
fn run_steps(loading_time: f64, hold: f64, rest_step: f64) -> f64 {
    (loading_time + hold) / (rest_step * LOADED_STEP_FACTOR)
}

/// A press's time over D4's, for runs of `steps` steps at `seconds_a_step`.
fn press_over_d4(steps: f64, seconds_a_step: f64) -> f64 {
    RUNS_PER_VERDICT * steps * seconds_a_step / D4_SECONDS
}

/// Lamé's λ at Poisson's ratio `poisson` for shear modulus `mu`.
fn lame_lambda(mu: f64, poisson: f64) -> f64 {
    mu * 2.0 * poisson / (1.0 - 2.0 * poisson)
}

/// The loop's stable step at the start of a run, at rest (§16e): the elastic top mode's step, re-estimated
/// with the viscosity at that step.
fn rest_step(model: &ExplicitModel, obstacle: &Obstacle) -> f64 {
    let executor = cpu::f64::CpuExecutor::new(model, obstacle).expect("the obstacle must be valid");
    Stepper::new(executor, StepperConfig::new(0.0), 0.0).dt()
}

/// §16e's accuracy check on `model` (in §16p's form, as `tube_release` runs it on the tube): the loop's
/// step over 0.9 of the critical step, converged, less 1; and the reference's own drift.
///
/// The reference is the fixed point of the same estimate at 6000 iterations in f64, `β` updated to `2/Δt_c`
/// until the step moves by at most 1e-4, with 4000 iterations agreeing to 1e-4.
fn step_error(model: &ExplicitModel, obstacle: &Obstacle) -> (f64, f64) {
    let mut reference = cpu::f64::CpuExecutor::new(model, obstacle).unwrap();
    let perturbation = reference.epsilon().sqrt() * reference.shortest_edge();
    let mut limit = |iterations: usize, weight: f64| {
        let top = reference.estimate_top_mode(iterations, perturbation, weight);
        let xi = top.damping_ratio();
        2.0 / top.omega_squared.sqrt() * (xi.mul_add(xi, 1.0).sqrt() - xi)
    };
    let mut step = limit(6000, 0.0);
    for _ in 0..8 {
        let next = limit(6000, 2.0 / step);
        let moved = next / step - 1.0;
        step = next;
        if moved.abs() <= 1e-4 {
            break;
        }
    }
    let drift = limit(4000, 2.0 / step) / step - 1.0;
    (rest_step(model, obstacle) / (0.9 * step) - 1.0, drift)
}

/// The f32 CPU executor's wall time a step on `model`, over [`TIMED_STEPS`] steps from rest. The obstacle is moved 1 m clear: every surface node still makes its lookups each
/// step, and none is in contact. With the scan at the path's start instead, a timed run's monitors went
/// non-finite by step 100 (why is not isolated; a run's pre-roll is step 6's).
fn cpu_seconds_a_step(model: &ExplicitModel, obstacle: &Obstacle) -> f64 {
    let mut clear = obstacle.clone();
    for pose in &mut clear.poses {
        pose.tx += 1.0;
    }
    let executor = cpu::f32::CpuExecutor::new(model, &clear).expect("the obstacle must be valid");
    let mut stepper = Stepper::new(executor, StepperConfig::new(0.0), 0.0);
    let started = Instant::now();
    for _ in 0..TIMED_STEPS {
        stepper.step().expect("a step at rest stays finite");
    }
    started.elapsed().as_secs_f64() / TIMED_STEPS as f64
}

/// A product mesh lowered into the explicit solver's model.
struct Lowered {
    model: ExplicitModel,
    /// Each model node's vertex in the source mesh. The mesher leaves lattice vertices that no element
    /// names; the model has no massless nodes, so it drops them.
    source: Vec<VertexId>,
}

/// Lower `mesh` into an [`ExplicitModel`]: its referenced vertices, its elements as the mesher orders them
/// (positive rest volume by construction; the model rejects any other), and per element its Yeoh `μ` and
/// `C₂`, λ at Poisson's ratio `poisson`, `densities[element]` and the Kelvin–Voigt viscosity
/// `viscous_time · μ`. No node is held.
///
/// # Errors
/// A [`ModelError`] if the model is rejected.
fn lower(
    mesh: &SdfMeshedTetMesh<Yeoh>,
    densities: &[f64],
    viscous_time: f64,
    poisson: f64,
) -> Result<Lowered, ModelError> {
    if densities.len() != mesh.n_tets() {
        return Err(ModelError::LengthMismatch {
            what: "densities",
            found: densities.len(),
            expected: mesh.n_tets(),
        });
    }
    let positions = mesh.positions();
    let point = |v: VertexId| {
        let p = positions[v as usize];
        [p.x, p.y, p.z]
    };
    let mut index = vec![None; positions.len()];
    let mut source = Vec::new();
    let mut elements = Vec::with_capacity(mesh.n_tets());
    let mut materials = Vec::with_capacity(mesh.n_tets());
    for (element, (yeoh, &density)) in mesh.materials().iter().zip(densities).enumerate() {
        let tet = TetId::try_from(element).map_err(|_| ModelError::TooLarge)?;
        let mut lowered = [0_u32; 4];
        for (slot, vertex) in lowered.iter_mut().zip(mesh.tet_vertices(tet)) {
            *slot = match index[vertex as usize] {
                Some(node) => node,
                None => {
                    let node = u32::try_from(source.len()).map_err(|_| ModelError::TooLarge)?;
                    index[vertex as usize] = Some(node);
                    source.push(vertex);
                    node
                }
            };
        }
        elements.push(lowered);
        materials.push(Material {
            mu: yeoh.mu(),
            lambda: lame_lambda(yeoh.mu(), poisson),
            c2: yeoh.c2(),
            viscosity: viscous_time * yeoh.mu(),
            density,
        });
    }
    let rest_positions = source.iter().map(|&v| point(v)).collect();
    let held = vec![false; source.len()];
    Ok(Lowered {
        model: ExplicitModel::new(rest_positions, elements, materials, held)?,
        source,
    })
}

/// Each element's density: its layer's catalog density.
fn densities(geometry: &InsertionGeometry, design: &cf_device_types::SimDesign) -> Vec<f64> {
    geometry
        .per_tet_layer
        .iter()
        .map(|&layer| cf_device_types::material_density(&design.layers[layer].anchor_key))
        .collect()
}

/// A surface's exact signed distance: a mesh's exact distance, signed by a flood-filled grid, negative
/// inside. The sign is read by interpolating the grid, so it is reliable only more than a cell from the
/// surface; the canal nodes sit an inset deep.
struct Truth {
    distance: TriMeshDistance,
    sign: CachedGridSdf,
}

impl Truth {
    fn signed(&self, p: Point3<f64>) -> f64 {
        let d = self.distance.distance(p);
        if self.sign.signed_distance(p) < 0.0 {
            -d
        } else {
            d
        }
    }

    /// Where `p` lands on the level `-inset`: steps of `p − (d + inset)·∇d` until within 1 nm of it, at most
    /// eight. One step can leave a node off the level where the nearest facet changes along the move.
    fn onto_level(&self, p: Point3<f64>, inset: f64) -> Point3<f64> {
        let mut p = p;
        for _ in 0..8 {
            let d = self.signed(p);
            if (d + inset).abs() <= 1e-9 {
                break;
            }
            let away = (p - self.distance.closest_point(p)).normalize();
            let gradient = if d < 0.0 { -away } else { away };
            p -= gradient * (d + inset);
        }
        p
    }
}

/// The canal nodes' offsets from the true canal surface, in element sizes: positive into the wall,
/// negative into the canal.
struct Bias {
    nodes: usize,
    /// The share of canal nodes inside the true canal surface by more than [`ON_THE_SURFACE`]: the wall
    /// reaching into the canal.
    inside: f64,
    /// The share outside it by more than [`ON_THE_SURFACE`]: the canal wider than designed.
    outside: f64,
    mean: f64,
    low: f64,
    high: f64,
    worst: f64,
}

impl Bias {
    /// Over `nodes` of `model`, against the level `-inset` of `truth`, in units of `size`.
    fn of(model: &ExplicitModel, nodes: &[u32], truth: &Truth, inset: f64, size: f64) -> Self {
        Self::from_offsets(
            nodes
                .iter()
                .map(|&n| {
                    let [x, y, z] = model.rest_positions()[n as usize];
                    (truth.signed(Point3::new(x, y, z)) + inset) / size
                })
                .collect(),
        )
    }

    fn from_offsets(mut offsets: Vec<f64>) -> Self {
        offsets.sort_by(f64::total_cmp);
        let count = offsets.len() as f64;
        let share =
            |keep: fn(f64) -> bool| offsets.iter().filter(|&&o| keep(o)).count() as f64 / count;
        Self {
            nodes: offsets.len(),
            inside: share(|o| o < -ON_THE_SURFACE),
            outside: share(|o| o > ON_THE_SURFACE),
            mean: offsets.iter().sum::<f64>() / count,
            low: quantile(&offsets, 0.05),
            high: quantile(&offsets, 0.95),
            worst: offsets.iter().fold(0.0, |w: f64, &o| w.max(o.abs())),
        }
    }

    fn line(&self) -> String {
        format!(
            "canal nodes {} [LOCAL] | inside the true surface {:.1} %, outside {:.1} % | offset/h mean {:+.3} p5 {:+.3} p95 {:+.3} worst {:.3}",
            self.nodes,
            100.0 * self.inside,
            100.0 * self.outside,
            self.mean,
            self.low,
            self.high,
            self.worst
        )
    }
}

/// Whether a point lies off every cap plane's plane, `(centroid, normal)`, by more than `margin`.
fn off_the_caps(planes: &[(Point3<f64>, Vector3<f64>)], margin: f64, p: Point3<f64>) -> bool {
    planes
        .iter()
        .all(|(centroid, normal)| (p - centroid).dot(&normal.normalize()).abs() > margin)
}

/// The model's canal nodes: its boundary nodes within [`CANAL_WINDOW`] element sizes of the true canal
/// surface, less those within one element size of a cap plane, where the cavity's flat floor meets it.
/// Also how many boundary nodes away from the caps lie in the next element size out: canal nodes the
/// selection would miss if the mesher's error reached that far.
fn canal_nodes(
    model: &ExplicitModel,
    truth: &Truth,
    caps: &[(Point3<f64>, Vector3<f64>)],
    inset: f64,
    size: f64,
) -> (Vec<u32>, usize) {
    let mut surface: Vec<u32> = model
        .surface_triangles()
        .iter()
        .flatten()
        .copied()
        .collect();
    surface.sort_unstable();
    surface.dedup();
    let offsets: Vec<(u32, f64)> = surface
        .into_iter()
        .filter_map(|n| {
            let [x, y, z] = model.rest_positions()[n as usize];
            let p = Point3::new(x, y, z);
            off_the_caps(caps, size, p).then(|| (n, (truth.signed(p) + inset).abs() / size))
        })
        .collect();
    let beyond = offsets
        .iter()
        .filter(|&&(_, o)| o > CANAL_WINDOW && o <= CANAL_WINDOW + 1.0)
        .count();
    let canal = offsets
        .into_iter()
        .filter(|&(_, o)| o <= CANAL_WINDOW)
        .map(|(n, _)| n)
        .collect();
    (canal, beyond)
}

/// A mesh's flood-fill-signed distance on a grid of spacing `cell` over `bounds`, in the explicit solver's
/// layout, pre-smoothed by `sigma_cells` (0: none; the old path's is [`GRID_SDF_SMOOTH_SIGMA_CELLS`]); and
/// the flood-filled grid itself.
fn scan_grid(
    distance: &TriMeshDistance,
    bounds: Aabb,
    cell: f64,
    sigma_cells: f64,
) -> (SdfGridLayout, Vec<f64>, CachedGridSdf) {
    let (cached, report) =
        CachedGridSdf::build(distance, bounds, cell, 0.75).expect("the scan's grid must build");
    let [w, h, d] = report.dims;
    let mut values = Vec::with_capacity(w * h * d);
    for k in 0..d {
        for j in 0..h {
            for i in 0..w {
                values.push(cached.signed_distance(Point3::new(
                    bounds.min.x + i as f64 * cell,
                    bounds.min.y + j as f64 * cell,
                    bounds.min.z + k as f64 * cell,
                )));
            }
        }
    }
    let values = gaussian_smooth_3d_separable(&values, w, h, d, sigma_cells);
    let size = |n: usize| u32::try_from(n).expect("a grid side fits a u32");
    let layout = SdfGridLayout {
        origin_x: bounds.min.x,
        origin_y: bounds.min.y,
        origin_z: bounds.min.z,
        cell_size: cell,
        size_x: size(w),
        size_y: size(h),
        size_z: size(d),
    };
    (layout, values, cached)
}

/// The scan as the solver's obstacle, held at `pose`.
fn scan_obstacle(grid: SdfGridLayout, values: Vec<f64>, pose: Pose) -> Obstacle {
    Obstacle {
        grid,
        values,
        start: 0.0,
        interval: 1.0,
        poses: vec![pose],
        friction: 0.0,
    }
}

/// G2's margin: the obstacle grid's distance at points on the scan, where it should read zero.
///
/// Where it reads positive, the grid's surface lies inside the scan's, and a node the contact law holds
/// on the grid's surface sits that deep in the scan: the penetration it allows. Where it reads negative, a
/// node stops that far short of the scan.
struct GridError {
    /// How many points were read.
    points: usize,
    /// The share of points where the grid allows a penetration deeper than `bar`.
    over_bar: f64,
    /// The allowed penetration's 95th percentile, 99th, 99.9th and largest, over `bar`.
    penetration: [f64; 4],
    /// The shortfall's 95th percentile and largest, over `bar`.
    gap: [f64; 2],
    /// Where the largest penetration is read.
    deepest: Point3<f64>,
}

/// [`GridError`] over the vertices `surface`'s faces name, and its face centroids, against `bar`.
fn grid_error_on_the_scan(obstacle: &Obstacle, surface: &IndexedMesh, bar: f64) -> GridError {
    let mut named: Vec<u32> = surface.faces.iter().flatten().copied().collect();
    named.sort_unstable();
    named.dedup();
    let centroids = surface.faces.iter().map(|face| {
        let [a, b, c] = face.map(|v| surface.vertices[v as usize].coords);
        Point3::from((a + b + c) / 3.0)
    });
    let readings: Vec<(Point3<f64>, f64)> = named
        .iter()
        .map(|&v| surface.vertices[v as usize])
        .chain(centroids)
        .map(|p| (p, obstacle.sample([p.x, p.y, p.z]).distance / bar))
        .collect();
    let sorted = |part: fn(f64) -> f64| {
        let mut values: Vec<f64> = readings.iter().map(|&(_, r)| part(r)).collect();
        values.sort_by(f64::total_cmp);
        values
    };
    let (penetration, gap) = (sorted(|r| r.max(0.0)), sorted(|r| (-r).max(0.0)));
    GridError {
        points: readings.len(),
        over_bar: readings.iter().filter(|&&(_, r)| r > 1.0).count() as f64 / readings.len() as f64,
        penetration: [
            quantile(&penetration, 0.95),
            quantile(&penetration, 0.99),
            quantile(&penetration, 0.999),
            quantile(&penetration, 1.0),
        ],
        gap: [quantile(&gap, 0.95), quantile(&gap, 1.0)],
        deepest: readings
            .iter()
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .map(|&(p, _)| p)
            .unwrap(),
    }
}

/// The product's wall meshed near `target` element size from an SDF source of `sdf_faces` faces: a first
/// mesh at the old path's 4 mm lattice, then secant steps on the lattice spacing until the element size is
/// within 2 % of `target` (at most three).
fn wall_at_size(
    scan: &IndexedMesh,
    design: &cf_device_types::SimDesign,
    caps: &[cf_cap_planes::CapPlane],
    target: f64,
    sdf_faces: usize,
) -> (InsertionGeometry, f64) {
    let mut cell = 0.004;
    for attempt in 0..4 {
        let geometry = build_insertion_geometry(scan, design, caps, sdf_faces, cell)
            .expect("the product's wall must mesh");
        let lowered = lower(&geometry.mesh, &densities(&geometry, design), 0.0, 0.49)
            .expect("the product's wall must lower");
        let size = element_size(&lowered.model);
        println!(
            "  mesh {attempt}: lattice {:.3} mm, h {:.3} mm ({:+.1} % of the target) [LOCAL]",
            1e3 * cell,
            1e3 * size,
            100.0 * (size / target - 1.0)
        );
        if (size / target - 1.0).abs() <= 0.02 || attempt == 3 {
            return (geometry, cell);
        }
        cell *= target / size;
    }
    unreachable!("the loop returns on its last attempt")
}

/// The product's loading: at the tube's rung speed in its innermost material, over its insertion path and
/// plan §15b's start gap, then §15b's hold. Returns the loading time and the hold.
fn product_loading(
    geometry: &InsertionGeometry,
    design: &cf_device_types::SimDesign,
    centerline: &[Point3<f64>],
) -> (f64, f64) {
    let plan = Insertion::plan(1.0);
    let innermost = geometry
        .mesh
        .materials()
        .iter()
        .zip(&geometry.per_tet_layer)
        .find(|&(_, &layer)| layer == 0)
        .map(|(yeoh, _)| yeoh.mu())
        .unwrap();
    let density = cf_device_types::material_density(&design.layers[0].anchor_key);
    let speed = speed_over_shear_wave() * (innermost / density).sqrt();
    let travel = polyline_arc_length_m(centerline) + plan.start_gap;
    let loading = travel / (0.9 * speed);
    println!(
        "loading: v/c_s {:.4} [PUBLIC]; v {:.3} m/s, travel {:.1} mm, loading {:.4} s, hold {} s [LOCAL]",
        speed_over_shear_wave(),
        speed,
        1e3 * travel,
        loading,
        plan.hold
    );
    (loading, plan.hold)
}

/// What one model costs: its rest step, a run's steps, and a press over D4 at K1's per-step budget
/// (`budget` on `k1_elements`) and on the CPU as timed. Prints one line; returns the rest step.
fn cost_line(
    label: &str,
    model: &ExplicitModel,
    obstacle: &Obstacle,
    (loading, hold): (f64, f64),
    (budget, k1_elements): (f64, usize),
    timed: bool,
) -> f64 {
    let step = rest_step(model, obstacle);
    let steps = run_steps(loading, hold, step);
    let at_k1 = press_over_d4(
        steps,
        budget * model.element_count() as f64 / k1_elements as f64,
    );
    let cpu = timed.then(|| press_over_d4(steps, cpu_seconds_a_step(model, obstacle)));
    println!(
        "{label}: per press / D4 at K1 {at_k1:.3}{} [PUBLIC]; rest step {:.3} us, {:.0} steps [LOCAL]",
        cpu.map_or_else(String::new, |c| format!(", on the CPU {c:.3}")),
        1e6 * step,
        steps
    );
    step
}

/// §16j's measurement of the product's budget. Prints a report; asserts nothing about the product.
///
/// `RAYON_NUM_THREADS=4 cargo test --release -p cf-sim-research explicit_budget -- --ignored --nocapture`,
/// on an idle machine (it times the CPU executor), with the scan at `~/scans/base_mold.cleaned.stl` (or
/// `CF_SIM_RESEARCH_PRODUCT_SCAN`). It fails when the scan is missing, rather than skipping (§16j).
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn the_products_budget_on_the_explicit_solver() {
    let (scan, centerline, caps, design) =
        super::tests::product_scene().expect("2d measures the product scan, which must be present");
    let inset = design.cavity_inset_m;
    let planes: Vec<(Point3<f64>, Vector3<f64>)> =
        caps.iter().map(cf_cap_planes::CapPlane::as_tuple).collect();

    println!("\n══ 2d: the product's budget on the explicit solver (§16j) ══");
    println!(
        "LOCAL lines stay on this machine; PUBLIC lines are ratios and verdicts for the plan. RAYON_NUM_THREADS={}",
        std::env::var("RAYON_NUM_THREADS").unwrap_or_else(|_| "unset".to_owned())
    );

    let (target, corner) = h_k2();
    println!(
        "h_K2 {:.3} mm, set by K2 at {corner} [PUBLIC]",
        1e3 * target
    );

    // The truths: the closed scan's distance for the obstacle and G2; the scan less its caps for the canal,
    // which the mesher offsets from the cap-stripped scan's distance near the mouth (`pinned_floor_shell`).
    // Both signed by the closed scan's 1 mm flood-filled grid, which is also the step estimates' obstacle.
    let bounds = scan_aabb(&scan, 0.004);
    let closed = TriMeshDistance::new(scan.clone()).expect("the scan's distance must build");
    let (grid_1mm, values_1mm, sign) = scan_grid(&closed, bounds, 0.001, 0.0);
    let canal_truth = Truth {
        distance: TriMeshDistance::new(cf_cap_planes::dome_wall_only_mesh(&scan, &caps))
            .expect("the cap-stripped scan's distance must build"),
        sign,
    };
    let start_pose = {
        let iso = slide_pose_at(&centerline, 0.0);
        let (q, t) = (iso.rotation, iso.translation.vector);
        Pose {
            qw: q.w,
            qx: q.i,
            qy: q.j,
            qz: q.k,
            tx: t.x,
            ty: t.y,
            tz: t.z,
        }
    };
    let obstacle = scan_obstacle(grid_1mm, values_1mm, start_pose);

    // K1's per-step budget: 2 minutes over the damped 100k tube's steps at the ladder's rung.
    let k1 = tube_run(TubeMesh::HundredK);
    let k1_tube = Tube::plan(TubeMesh::HundredK);
    let k1_model = k1_tube.model(k1.material(), Walls::Free).unwrap();
    let k1_step = rest_step(&k1_model, &k1.obstacle(&k1_tube).unwrap());
    let k1_steps = run_steps(k1.insertion.loading_time, k1.insertion.hold, k1_step);
    let k1 = (K1_SECONDS / k1_steps, k1_model.element_count());
    println!(
        "K1: 100k tube at {LOADING_RUNG} T_s, rest step {:.3} us, {:.0} steps ⇒ {:.3} ms a step on {} elements [PUBLIC: the tube]",
        1e6 * k1_step,
        k1_steps,
        1e3 * k1.0,
        k1.1
    );

    let (geometry, cell) = wall_at_size(&scan, &design, &caps, target, 2_500);
    let densities = densities(&geometry, &design);
    let loading = product_loading(&geometry, &design, &centerline);
    let wall = |poisson: f64, viscous_time: f64| {
        lower(&geometry.mesh, &densities, viscous_time, poisson).unwrap()
    };
    let elastic = wall(0.49, 0.0);
    let size = element_size(&elastic.model);
    println!(
        "wall: h/h_K2 {:.3} [PUBLIC]; lattice {:.3} mm, {} elements, {} nodes, {} unreferenced vertices dropped [LOCAL]",
        size / target,
        1e3 * cell,
        elastic.model.element_count(),
        elastic.model.node_count(),
        geometry.mesh.positions().len() - elastic.model.node_count(),
    );

    // The budget, at each ν, elastic and at the viscosity's range; the CPU timed at ν 0.49.
    let mut steps = Vec::new();
    for poisson in PRODUCT_POISSON {
        let at = |label: String, viscous_time: f64, timed: bool| {
            cost_line(
                &label,
                &wall(poisson, viscous_time).model,
                &obstacle,
                loading,
                k1,
                timed,
            )
        };
        let elastic_step = at(format!("ν {poisson}, η off"), 0.0, poisson == 0.49);
        for scale in VISCOSITY_SCALES {
            let viscous_step = at(
                format!("ν {poisson}, η ×{scale:.3} of Ecoflex 00-30's η/μ"),
                scale * ECOFLEX_00_30_VISCOUS_TIME,
                poisson == 0.49 && scale == 1.0,
            );
            if scale == 1.0 {
                steps.push((poisson, elastic_step, viscous_step));
            }
        }
    }
    for (poisson, elastic_step, viscous_step) in &steps {
        println!(
            "ν {poisson}: the viscosity's step over the elastic step {:.4} [PUBLIC]",
            viscous_step / elastic_step
        );
    }

    // §16e: the step's accuracy on the product's mesh, before the budget relies on it.
    let viscous = wall(0.49, ECOFLEX_00_30_VISCOUS_TIME);
    for (label, model) in [
        ("as meshed, ν 0.49, η off", &elastic.model),
        ("as meshed, ν 0.49, η on", &viscous.model),
    ] {
        let (error, drift) = step_error(model, &obstacle);
        println!(
            "step accuracy, {label}: the loop's step over 0.9 of the converged critical step {error:+.4} (bar ±{STEP_ACCURACY_BAR}; reference drift {drift:+.1e}) [PUBLIC]"
        );
    }

    // The surface bias, and what projecting the canal nodes onto the true surface does.
    let (canal, beyond) = canal_nodes(&elastic.model, &canal_truth, &planes, inset, size);
    let before = Bias::of(&elastic.model, &canal, &canal_truth, inset, size);
    println!(
        "surface bias, as meshed: {} | boundary nodes {CANAL_WINDOW}–{} h from the level {beyond} [PUBLIC but the counts]",
        before.line(),
        CANAL_WINDOW + 1.0
    );
    let as_meshed = steps[0];
    for floor in PROJECTION_FLOORS {
        let moves: Vec<(VertexId, Vec3)> = canal
            .iter()
            .map(|&n| {
                let [x, y, z] = elastic.model.rest_positions()[n as usize];
                let target = canal_truth.onto_level(Point3::new(x, y, z), inset);
                (elastic.source[n as usize], target.coords)
            })
            .collect();
        let projected = geometry.mesh.clone().with_projected_nodes(&moves, floor);
        let lowered =
            |viscous_time: f64| lower(&projected, &densities, viscous_time, 0.49).unwrap();
        let (projected_elastic, projected_viscous) =
            (lowered(0.0), lowered(ECOFLEX_00_30_VISCOUS_TIME));
        assert_eq!(
            projected_elastic.source, elastic.source,
            "projecting moved no topology"
        );
        let after = Bias::of(&projected_elastic.model, &canal, &canal_truth, inset, size);
        let reached = canal
            .iter()
            .filter(|&&n| {
                let [x, y, z] = projected_elastic.model.rest_positions()[n as usize];
                (canal_truth.signed(Point3::new(x, y, z)) + inset).abs() <= ON_THE_SURFACE * size
            })
            .count();
        println!(
            "projected, floor {floor}: {} | reached within {ON_THE_SURFACE} h {:.1} % [PUBLIC but the count]",
            after.line(),
            100.0 * reached as f64 / canal.len() as f64,
        );
        let elastic_step = cost_line(
            &format!("projected, floor {floor}, ν 0.49, η off"),
            &projected_elastic.model,
            &obstacle,
            loading,
            k1,
            true,
        );
        let viscous_step = cost_line(
            &format!("projected, floor {floor}, ν 0.49, η on"),
            &projected_viscous.model,
            &obstacle,
            loading,
            k1,
            true,
        );
        println!(
            "projected, floor {floor}: the step over as meshed, η off {:.4}, η on {:.4} [PUBLIC]",
            elastic_step / as_meshed.1,
            viscous_step / as_meshed.2
        );
        for (label, model) in [
            ("η off", &projected_elastic.model),
            ("η on", &projected_viscous.model),
        ] {
            let (error, drift) = step_error(model, &obstacle);
            println!(
                "step accuracy, projected at floor {floor}, ν 0.49, {label}: {error:+.4} (drift {drift:+.1e}) [PUBLIC]"
            );
        }
    }

    // What the surface bias owes to the SDF source: the same lattice from the undecimated scan.
    let full = build_insertion_geometry(&scan, &design, &caps, scan.faces.len(), cell)
        .expect("the wall must mesh from the full scan");
    let full_model = lower(&full.mesh, &self::densities(&full, &design), 0.0, 0.49)
        .unwrap()
        .model;
    let full_size = element_size(&full_model);
    let (full_canal, full_beyond) =
        canal_nodes(&full_model, &canal_truth, &planes, inset, full_size);
    println!(
        "surface bias from the undecimated scan at the same lattice (h/h_K2 {:.3}): {} | {full_beyond} beyond [PUBLIC but the counts]",
        full_size / target,
        Bias::of(&full_model, &full_canal, &canal_truth, inset, full_size).line()
    );

    // The budget if D1 needs the 50k tube's element size (K5, fit plan U16).
    let fine_target = tube_size(TubeMesh::FiftyK);
    let (fine, _) = wall_at_size(&scan, &design, &caps, fine_target, 2_500);
    let fine_densities = self::densities(&fine, &design);
    for (label, viscous_time) in [("η off", 0.0), ("η on", ECOFLEX_00_30_VISCOUS_TIME)] {
        let model = lower(&fine.mesh, &fine_densities, viscous_time, 0.49)
            .unwrap()
            .model;
        cost_line(
            &format!(
                "at the 50k tube's h (h/h_50k {:.3}), ν 0.49, {label}",
                element_size(&model) / fine_target
            ),
            &model,
            &obstacle,
            loading,
            k1,
            true,
        );
    }

    // G2's margin: the obstacle grid's error on the scan, unsmoothed and with the old path's pre-smooth;
    // over every point, and over all but the cap discs, the flat faces the canal never meets (the faces
    // `dome_wall_only_mesh` strips, by their normal and their distance from a cap plane).
    let bar = G2_FRACTION_OF_INSET * inset;
    let sides = cf_cap_planes::dome_wall_only_mesh(&scan, &caps);
    println!(
        "G2 points: {} faces of {} are cap faces [LOCAL]",
        scan.faces.len() - sides.faces.len(),
        scan.faces.len()
    );
    for cell in [0.001, 0.0005, 0.00025] {
        for sigma in [0.0, GRID_SDF_SMOOTH_SIGMA_CELLS] {
            let (grid, values, _) = scan_grid(&closed, bounds, cell, sigma);
            let grid_obstacle = scan_obstacle(grid, values, start_pose);
            let all = grid_error_on_the_scan(&grid_obstacle, &scan, bar);
            let off_the_discs = grid_error_on_the_scan(&grid_obstacle, &sides, bar);
            for (label, error) in [
                ("all points", all),
                ("all but the cap discs", off_the_discs),
            ] {
                let from_the_caps = planes
                    .iter()
                    .map(|(c, n)| (error.deepest - c).dot(&n.normalize()).abs())
                    .fold(f64::INFINITY, f64::min);
                println!(
                    "G2 grid {:.2} mm, pre-smooth {sigma} cell, {label}: deeper than the bar at {:.2} % | penetration over the bar p95 {:.3} p99 {:.3} p99.9 {:.3} worst {:.3}, {:.2} h_K2 from a cap plane | shortfall p95 {:.3} worst {:.3} [PUBLIC]; {} points [LOCAL]",
                    1e3 * cell,
                    100.0 * error.over_bar,
                    error.penetration[0],
                    error.penetration[1],
                    error.penetration[2],
                    error.penetration[3],
                    from_the_caps / target,
                    error.gap[0],
                    error.gap[1],
                    error.points
                );
            }
        }
    }
}

#[test]
fn size_at_bar_recovers_a_power_law() {
    let (c, p) = (3.0, 2.2);
    let error = |h: f64| c * h.powf(p);
    let (coarse, fine) = (0.4, 0.15);
    let h = size_at_bar((coarse, error(coarse)), (fine, error(fine)), 0.2);
    assert!((error(h) / 0.2 - 1.0).abs() < 1e-12, "e(h) = {}", error(h));
}

#[test]
fn h_k2_is_set_by_the_worst_corner() {
    let (h, corner) = h_k2();
    assert_eq!(corner, "λ_a 1.1, ν 0.495");
    assert!(
        (h - 2.198e-3).abs() < 1e-6,
        "h_K2 {h}: §16p's inputs put it at 2.198 mm"
    );
}

#[test]
fn the_stop_rules_element_size_is_the_mean_volumes_cube_root() {
    let rest = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, -2.0],
    ];
    // Volumes 1/6 and 2/6: a mean of 1/4.
    let elements = vec![[0, 1, 2, 3], [0, 2, 1, 4]];
    let material = tube_run(TubeMesh::TenK).material();
    let model = ExplicitModel::new(rest, elements, vec![material; 2], vec![false; 5]).unwrap();
    assert!((element_size(&model) - 0.25_f64.cbrt()).abs() < 1e-15);
}

#[test]
fn the_budget_arithmetic() {
    // Plan §15b's travel, 105 mm, at its constant speed over 90 % of 0.625 of T_s = 0.48 s at c_s = 1.
    assert!((speed_over_shear_wave() - 0.105 / (0.9 * 0.3)).abs() < 1e-15);
    let steps = run_steps(0.1, 0.2, 1e-4);
    assert!((steps - 0.3 / (1e-4 * 0.977)).abs() < 1e-9);
    // Three runs of 1000 steps at 0.1 s a step: 300 s, one D4.
    assert!((press_over_d4(1000.0, 0.1) - 1.0).abs() < 1e-15);
    assert!((lame_lambda(1.0, 0.49) - 49.0).abs() < 1e-12);
}

#[test]
fn the_bias_reads_its_offsets() {
    // 41 offsets, −1.0 to +1.0 by 0.05: nearest rank puts the 5th and 95th percentiles at the 3rd and 39th.
    let offsets: Vec<f64> = (0..41).map(|i| f64::from(i) * 0.05 - 1.0).collect();
    let bias = Bias::from_offsets(offsets.iter().rev().copied().collect());
    assert_eq!(bias.nodes, 41);
    assert!((bias.inside - 20.0 / 41.0).abs() < 1e-15, "{}", bias.inside);
    assert!(
        (bias.outside - 20.0 / 41.0).abs() < 1e-15,
        "{}",
        bias.outside
    );
    assert!(bias.mean.abs() < 1e-12, "{}", bias.mean);
    assert!((bias.low - (-0.9)).abs() < 1e-12 && (bias.high - 0.9).abs() < 1e-12);
    assert!((bias.worst - 1.0).abs() < 1e-12);
}

#[test]
fn only_points_on_a_cap_plane_are_on_it() {
    // A normal that is not unit length: 6 mm off the plane is within 10 mm of it, not 12 mm off.
    let planes = [(Point3::new(0.0, 0.0, 0.1), Vector3::new(0.0, 0.0, 2.0))];
    assert!(!off_the_caps(&planes, 1e-5, Point3::new(0.3, -0.2, 0.1)));
    assert!(off_the_caps(&planes, 1e-5, Point3::new(0.0, 0.0, 0.1001)));
    assert!(!off_the_caps(&planes, 0.01, Point3::new(0.0, 0.0, 0.094)));
}

/// A two-layer wall on the icosphere the old path's gates use (`tolerance_fixture`), at their 4 mm lattice:
/// two materials, so a lowering that mixes up elements shows.
fn synthetic_wall() -> (InsertionGeometry, cf_device_types::SimDesign) {
    let layer = |anchor: &str| cf_device_types::SimLayer {
        thickness_m: 0.004,
        anchor_key: anchor.to_owned(),
        slacker_fraction: 0.0,
    };
    let design = cf_device_types::SimDesign {
        cavity_inset_m: 0.003,
        layers: vec![layer("ECOFLEX_00_30"), layer("DRAGON_SKIN_20A")],
    };
    let geometry = build_insertion_geometry(
        &super::tests::icosphere(0.020, 2),
        &design,
        &[],
        2_000,
        0.004,
    )
    .expect("the synthetic wall must mesh");
    (geometry, design)
}

#[test]
fn lowering_keeps_every_element_and_drops_only_unreferenced_vertices() {
    let (geometry, design) = synthetic_wall();
    let mesh = &geometry.mesh;
    let densities = densities(&geometry, &design);
    let lowered = lower(mesh, &densities, ECOFLEX_00_30_VISCOUS_TIME, 0.49).unwrap();
    let model = &lowered.model;

    let mut referenced: Vec<VertexId> = (0..mesh.n_tets())
        .flat_map(|t| mesh.tet_vertices(TetId::try_from(t).unwrap()))
        .collect();
    referenced.sort_unstable();
    referenced.dedup();
    assert!(
        referenced.len() < mesh.positions().len(),
        "the fixture must leave unreferenced vertices, or dropping them is not exercised"
    );
    assert!(
        densities.iter().any(|&d| d != densities[0]),
        "the fixture must carry two materials, or a mixed-up element is not seen"
    );
    assert_eq!(model.node_count(), referenced.len());
    assert_eq!(model.element_count(), mesh.n_tets());

    for (element, corners) in model.elements().iter().enumerate() {
        let source = mesh.tet_vertices(TetId::try_from(element).unwrap());
        let mut names: Vec<VertexId> = corners.map(|n| lowered.source[n as usize]).to_vec();
        let mut expected = source.to_vec();
        names.sort_unstable();
        expected.sort_unstable();
        assert_eq!(names, expected, "element {element} names other vertices");
        let (material, yeoh) = (model.materials()[element], &mesh.materials()[element]);
        assert_eq!(
            [material.mu, material.c2, material.density],
            [yeoh.mu(), yeoh.c2(), densities[element]]
        );
        assert!((material.lambda / yeoh.mu() - 49.0).abs() < 1e-12, "ν 0.49");
        assert!(
            (material.viscosity / (ECOFLEX_00_30_VISCOUS_TIME * yeoh.mu()) - 1.0).abs() < 1e-15
        );
    }
    for (node, &vertex) in lowered.source.iter().enumerate() {
        let p = mesh.positions()[vertex as usize];
        assert_eq!(model.rest_positions()[node], [p.x, p.y, p.z]);
    }
}

/// An axis-aligned box of half-sides `half`, wound outward.
fn a_box(half: [f64; 3]) -> IndexedMesh {
    let mut mesh = IndexedMesh::new();
    for k in [-1.0, 1.0] {
        for j in [-1.0, 1.0] {
            for i in [-1.0, 1.0] {
                mesh.vertices
                    .push(Point3::new(i * half[0], j * half[1], k * half[2]));
            }
        }
    }
    mesh.faces = vec![
        [0, 2, 1],
        [1, 2, 3],
        [4, 5, 6],
        [5, 7, 6],
        [0, 1, 4],
        [1, 5, 4],
        [2, 6, 3],
        [3, 6, 7],
        [0, 4, 2],
        [2, 4, 6],
        [1, 3, 5],
        [3, 7, 5],
    ];
    mesh
}

/// A mesh's [`Truth`], signed by a 1 mm grid.
fn truth_of(mesh: &IndexedMesh) -> Truth {
    let distance = TriMeshDistance::new(mesh.clone()).unwrap();
    let (_, _, sign) = scan_grid(&distance, scan_aabb(mesh, 0.004), 0.001, 0.0);
    Truth { distance, sign }
}

#[test]
fn a_node_moved_onto_the_level_sits_an_inset_deep() {
    let inset = 0.005;
    // Near a box's edge the nearest face changes along the move: one step lands 3 mm deep, not 5.
    let truth = truth_of(&a_box([0.020, 0.020, 0.020]));
    assert!(
        truth.signed(Point3::origin()) < 0.0 && truth.signed(Point3::new(0.03, 0.0, 0.0)) > 0.0
    );
    let p = Point3::new(-0.018, -0.017, 0.0);
    let depth = truth.signed(truth.onto_level(p, inset));
    assert!(
        (depth + inset).abs() < 1e-7,
        "near the edge the node lands {depth} from the surface"
    );
    let sphere = truth_of(&super::tests::icosphere(0.020, 4));
    for p in [
        Point3::new(0.0, 0.0, 0.0125),
        Point3::new(0.010, -0.006, 0.007),
    ] {
        let depth = sphere.signed(sphere.onto_level(p, 0.003));
        assert!(
            (depth + 0.003).abs() < 1e-7,
            "from {p:?} the node lands {depth} from the surface"
        );
    }
}

#[test]
fn the_canal_nodes_are_the_inner_surfaces() {
    let (geometry, design) = synthetic_wall();
    let model = lower(&geometry.mesh, &densities(&geometry, &design), 0.0, 0.49)
        .unwrap()
        .model;
    let truth = truth_of(&super::tests::icosphere(0.020, 2));
    let (canal, _) = canal_nodes(&model, &truth, &[], 0.003, element_size(&model));
    let mut inner: Vec<u32> = model
        .surface_triangles()
        .iter()
        .flatten()
        .copied()
        .filter(|&n| {
            let [x, y, z] = model.rest_positions()[n as usize];
            x.hypot(y).hypot(z) < 0.020
        })
        .collect();
    inner.sort_unstable();
    inner.dedup();
    assert!(!inner.is_empty());
    assert_eq!(canal, inner);
}

#[test]
fn the_grid_reads_its_mesh_and_g2_its_sign() {
    // A box, not a sphere, so a transposed grid shows.
    let mesh = a_box([0.010, 0.015, 0.025]);
    let distance = TriMeshDistance::new(mesh.clone()).unwrap();
    let (grid, values, _) = scan_grid(&distance, scan_aabb(&mesh, 0.004), 0.001, 0.0);
    let bar = 1e-4;
    let pose = Pose {
        qw: 1.0,
        qx: 0.0,
        qy: 0.0,
        qz: 0.0,
        tx: 0.0,
        ty: 0.0,
        tz: 0.0,
    };
    // A vertex no face names, well off the box, where the grid reads far from zero: not a point on the mesh.
    let mut with_a_stray = mesh.clone();
    with_a_stray.vertices.push(Point3::new(0.0, 0.0, 0.028));
    let on = grid_error_on_the_scan(
        &scan_obstacle(grid, values.clone(), pose),
        &with_a_stray,
        bar,
    );
    assert!(
        on.penetration[3] < 1e-6 / bar && on.gap[1] < 1e-6 / bar,
        "the grid reads its own mesh's faces at zero: {:?} {:?}",
        on.penetration,
        on.gap
    );
    // Lifting every value lifts the grid's surface inside the mesh: an allowed penetration everywhere.
    let lifted: Vec<f64> = values.iter().map(|v| v + 3.0 * bar).collect();
    let error = grid_error_on_the_scan(&scan_obstacle(grid, lifted, pose), &mesh, bar);
    assert!((error.over_bar - 1.0).abs() < 1e-15);
    assert!(
        error
            .penetration
            .iter()
            .all(|&p| (p - 3.0).abs() < 1e-6 / bar),
        "{:?}",
        error.penetration
    );
    assert!(error.gap[1] < 1e-12);
}
