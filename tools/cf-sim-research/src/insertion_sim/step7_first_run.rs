//! Step 7's first run on the product scan (soft-contact recon §16x): one press at the 5 mm inset, mounted at the
//! closed end, on the fitted path, with the rules §16x set before its runs. Six stages and three diagnostics, each an
//! ignored test:
//! - [`step7_ladder`], rule 1: the loading time, at h_K2;
//! - [`step7_sizes`], rule 2: the element size, at a loading, and rule 1 again at the size it picks (§16z: with an
//!   eight-times wall, and a replicate at four times);
//! - [`step7_at_h_k2`], rules 3, 5, 10 and 11 at h_K2 and a loading, with the push's linearity in friction;
//! - [`step7_room`], the room on step 7's wall;
//! - [`step7_cost`], G6: a press timed at a loading and a size, with the probe's own instruments off;
//! - [`step7_blow_up`] and [`step7_stiffening`], the element collapsing at the seated tip, one change at a time;
//! - [`step7_stabilized`], exploratory: the volumetric stabilization's ladder against the collapse and D1's readings
//!   (§16y);
//! - [`step7_masked`], §16y rule 2: the stabilization on the collapsing elements alone, against the element as it is;
//!   §16z runs it up to 25μ and prints U20's reading and the two sides of U11's verdict it bears on.
//!
//! A comparison reads its two runs at one re-estimate interval (§16z): where §16y's rule 1 ran one every 50 steps and
//! the other stood at the loop's 500, the other is run again every 50 steps ([`at_one_interval`]).
//!
//! Every run prints D1's readings, the sideways force and twist, G1 and G2 against the scan's exact distance, the band's
//! monitors, the validity gates and K4, and the seated window and contact work in quarters of the hold (rules 6–9). A
//! run whose readings do not stand ([`falls_short`]) feeds no rule, and a rule that would read it prints "not judged";
//! rule 10's probe holds are not gated.
//!
//! A stage's loading is `STEP7_LOADING`, in multiples of the budget's (default 1; §16x's runs used 4 for every stage but
//! the ladder); `step7_cost`'s size is `STEP7_SIZE`, 0, 1, 2 or 3 for one, two, four and eight times h_K2's element
//! count (default 0). Run each with
//! `RAYON_NUM_THREADS=4 cargo test --release -p cf-sim-research --bin cf-sim-research --
//! insertion_sim::step7_first_run::<stage> --ignored --nocapture`, the scan at `~/scans/base_mold.cleaned.stl` (or
//! `CF_SIM_RESEARCH_PRODUCT_SCAN`).
//!
//! ⛔ The scan never enters the repo. LOCAL lines stay on the machine that ran it; PUBLIC lines are the ratios and
//! verdicts the plan records (Jon, 2026-09-26).

#![cfg(test)]
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::too_many_lines
)]

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;
use std::time::Instant;

use cf_cap_planes::CapPlane;
use cf_device_types::SimDesign;
use mesh_types::IndexedMesh;
use nalgebra::{Isometry3, Matrix4, Point3, Translation3, UnitQuaternion, Vector3, Vector4};
use sim_gpu::GpuContext;
use sim_gpu::soft::GpuExecutor;
use sim_soft::lowering::path::{Centreline, FittedPath, travelled};
use sim_soft::lowering::{
    Lowered, Lowering, PRODUCT_BAKE, Plane, SAMPLING_BAR, START_CLEARANCE, Skin, lower,
};
use sim_soft::obstacle::{SignedDistance, bake_surface};
use sim_soft::pairing::PAIRINGS;
use sim_soft::{Mesh, SdfMeshedTetMesh, VertexId, Yeoh};
use sim_soft_explicit::ExplicitModel;
use sim_soft_explicit::cpu;
use sim_soft_explicit::executor::{Executor, Monitors, Obstacle, PhaseOutputs, Snapshot, TopMode};
use sim_soft_explicit::f64::{Material, Pose, pose_to_body};
use sim_soft_explicit::fixtures::tube::ECOFLEX_00_30_VISCOUS_TIME;
use sim_soft_explicit::readings::{
    PROBE_AREA, PUSH_TRAVEL, WindowContact, along_path, area_percentile, moment_about, sideways,
    static_push, work_peak,
};
use sim_soft_explicit::stepping::{RunError, Sample, Stepper, StepperConfig, gates};

use super::canal_surface::exact_fields;
use super::explicit_budget::{
    D4_SECONDS, element_size, g2_bar, h_k2, k1_budget, rest_step, speed_over_shear_wave,
};
use super::product_lowering::{
    HOLD, Less, POISSON, boundary_points, densities, pieces, step7_wall, step7_wall_at_size,
};

/// The pairing the first run presses at: the library's lowest `μ_f`, every corner checked (§16x).
const PAIRING: &str = "silicone on skin, water-based gel, fresh";

/// Rule 1's rungs, in multiples of the budget's loading time.
const RUNGS: [f64; 5] = [1.0, 2.0, 4.0, 8.0, 16.0];

/// K5's bar, which rules 1–3 and 10 borrow (§16x).
const K5_BAR: f64 = 0.05;

/// K3's bar, which rule 5 borrows.
const K3_BAR: f64 = 0.005;

/// Rule 6: a quarter of the hold is settled within this of the last quarter's patch.
const SETTLE_BAR: f64 = 0.01;

/// Rule 9: the work gained over the hold's last half, over the internal energy at the seat.
const PUMPING_BAR: f64 = 0.01;

/// Rule 11: every force within this of twice, 15d.10's bar.
const SCALING_BAR: f64 = 0.02;

/// Rule 2's element counts, over h_K2's; eight times judges four times' doubling (§16z).
const SIZES: [f64; 4] = [1.0, 2.0, 4.0, 8.0];

/// The sizes rule 2 runs a replicate at, on a lattice shifted half a cell, as indices into [`SIZES`]: h_K2's (§16x),
/// and four times its elements, whose doubling decides if any does (§16z).
const REPLICATES: [usize; 2] = [0, 2];

/// Rule 2's deciding readings (§16x): the patch at each corner and the geometric share, with the corner each is read at.
const DECIDING: [(&str, usize); 4] = [
    ("patch at 0", 0),
    ("patch at the lowest", 1),
    ("patch at the highest", 2),
    ("geometric share", 0),
];

/// Rule 3's Poisson's ratios.
const POISSONS: [f64; 3] = [0.49, 0.495, 0.4975];

/// A read reaches at most this much travel at the top speed (§16x).
const READ_TRAVEL: f64 = 0.000_25;

/// D1's peak push is the largest mean over this much travel; the windows beside it are printed too (§16x).
const PEAK_WINDOW: f64 = 0.001;
const PEAK_BESIDE: [f64; 2] = [0.000_5, 0.002];

/// Rule 10: the probe step, and each probe's move, hold and read.
const PROBE_STEP: f64 = 0.000_1;
const MOVE_TIME: f64 = 0.05;
const PROBE_HOLD: f64 = 0.1;
const PROBE_READ: f64 = 0.05;

/// The product scene, its fitted path and its exact distance.
struct Scene {
    scan: IndexedMesh,
    centerline: Vec<Point3<f64>>,
    caps: Vec<CapPlane>,
    design: SimDesign,
    path: FittedPath,
    distance: SignedDistance,
    /// The seated tip, in the scan's frame.
    tip: [f64; 3],
}

impl Scene {
    fn load() -> Self {
        let (scan, centerline, caps, design) = super::tests::product_scene()
            .expect("step 7 runs on the product scan, which must be present");
        let device: Vec<Plane> = caps
            .iter()
            .map(|cap| Plane::new(cap.centroid, cap.normal).unwrap())
            .collect();
        let path = FittedPath::new(&scan, Centreline::new(&centerline).unwrap(), device).unwrap();
        let distance = SignedDistance::new(&scan).unwrap();
        let tip = [centerline[0].x, centerline[0].y, centerline[0].z];
        Self {
            scan,
            centerline,
            caps,
            design,
            path,
            distance,
            tip,
        }
    }

    fn length(&self) -> f64 {
        self.path.centreline().length()
    }

    fn inset(&self) -> f64 {
        self.design.cavity_inset_m
    }
}

/// A wall of step 7's: its mesh, the vertices the mount holds, and where it sits.
struct Wall {
    mesh: SdfMeshedTetMesh<Yeoh>,
    mount: Vec<VertexId>,
    skin: usize,
    densities: Vec<f64>,
    cell: f64,
    /// The lattice's shift: the wall was meshed from the scene moved by this, and is moved back.
    shift: [f64; 3],
}

impl Wall {
    /// Step 7's wall near element size `target`, found by the secant, or at lattice `cell`; meshed from the scene moved
    /// by `shift` and moved back, so its lattice sits `shift` off the unshifted one's (the mesher's lattice is fixed in
    /// the world).
    fn build(scene: &Scene, target: f64, cell: Option<f64>, shift: [f64; 3]) -> Self {
        let by = Vector3::from(shift);
        let scan = IndexedMesh {
            vertices: scene.scan.vertices.iter().map(|v| v + by).collect(),
            faces: scene.scan.faces.clone(),
        };
        let caps: Vec<CapPlane> = scene
            .caps
            .iter()
            .map(|cap| CapPlane {
                centroid: cap.centroid + by,
                ..cap.clone()
            })
            .collect();
        let (mesh, cell) = match cell {
            Some(cell) => (step7_wall(&scan, &scene.design, &caps, cell), cell),
            None => step7_wall_at_size(&scan, &scene.design, &caps, target),
        };
        let (closed, _) = exact_fields(&scan, &caps);
        let total: f64 = scene.design.layers.iter().map(|l| l.thickness_m).sum();
        let outer = Less {
            field: closed,
            by: total - scene.inset(),
        };
        let skin = Skin::of(&mesh, &outer, cell);
        let track = Centreline::new(&scene.centerline).unwrap();
        let seated = track.tangent(0.0);
        let mount = skin.beyond(
            &mesh,
            &Plane::new(scene.centerline[0] + by, -seated).unwrap(),
        );
        let densities = densities(&scene.design, &mesh);
        Self {
            mesh,
            mount,
            skin: skin.vertices().len(),
            densities,
            cell,
            shift,
        }
    }

    /// The wall lowered at `poisson`, with Ecoflex 00-30's `η/μ` scaled by `viscous`, its moduli scaled by
    /// `stiffness` (`η/μ` held), mounted, and moved back by its shift.
    fn model(&self, poisson: f64, viscous: f64, stiffness: f64) -> ExplicitModel {
        let lowering = Lowering {
            poisson,
            viscous_time: viscous * ECOFLEX_00_30_VISCOUS_TIME,
        };
        let Lowered { model, .. } =
            lower(&self.mesh, &self.densities, lowering, &self.mount).unwrap();
        if self.shift == [0.0; 3] && stiffness == 1.0 {
            return model;
        }
        let back = self.shift;
        ExplicitModel::new(
            model
                .rest_positions()
                .iter()
                .map(|p| [p[0] - back[0], p[1] - back[1], p[2] - back[2]])
                .collect(),
            model.elements().to_vec(),
            model
                .materials()
                .iter()
                .map(|m| Material {
                    mu: stiffness * m.mu,
                    lambda: stiffness * m.lambda,
                    c2: stiffness * m.c2,
                    viscosity: stiffness * m.viscosity,
                    density: m.density,
                })
                .collect(),
            model.held().to_vec(),
        )
        .unwrap()
    }

    /// The innermost material's shear wave speed.
    fn shear_speed(&self) -> f64 {
        (self.mesh.materials()[0].mu() / self.densities[0]).sqrt()
    }
}

/// A run's settings.
#[derive(Clone, Copy, Debug)]
struct Spec {
    friction: f64,
    /// The loading time, seconds.
    loading: f64,
    wide: bool,
    /// Whether the run is on the GPU executor, at f32 (§17b), rather than the CPU's.
    gpu: bool,
    hold: f64,
    /// Whether the probe's instruments run: the exact G1 and G2, the windows' snapshots, and the collapse's reads.
    instruments: bool,
    /// Steps between the loop's re-estimates of the stable step (§15c: 500).
    reestimate_every: u64,
}

/// The elements collapsing against their nodes over a run's reads (§16y): the least element `J` over its nodes'
/// averaged `J`, and every element under half of it at some read.
#[derive(Clone, Debug)]
struct Collapse {
    least: f64,
    elements: BTreeSet<usize>,
}

impl Default for Collapse {
    fn default() -> Self {
        Self {
            least: f64::INFINITY,
            elements: BTreeSet::new(),
        }
    }
}

/// One monitor read, with where the path was and how deep the deepest node near the scan lay against its exact
/// distance.
#[derive(Clone, Copy, Debug)]
struct Read {
    sample: Sample,
    travel: f64,
    exact_depth: f64,
}

/// Wall-clock seconds: the executor's setup, the stepping, and the probe's own instruments.
#[derive(Clone, Copy, Debug, Default)]
struct Clock {
    setup: f64,
    stepping: f64,
    instruments: f64,
}

/// What a run is pressed against, and how its path maps time to travel.
struct Press<'a> {
    scene: &'a Scene,
    model: &'a ExplicitModel,
    surface: Vec<u32>,
    /// The walk at time 0.
    start: f64,
    loading: f64,
}

impl Press<'_> {
    fn travel(&self, time: f64) -> f64 {
        travelled(time / self.loading, self.start)
    }

    /// The time the path has travelled `travel`, by bisection.
    fn time_at(&self, travel: f64) -> f64 {
        let (mut low, mut high) = (0.0, self.loading);
        for _ in 0..60 {
            let mid = 0.5 * (low + high);
            if self.travel(mid) < travel {
                low = mid;
            } else {
                high = mid;
            }
        }
        0.5 * (low + high)
    }

    /// The deepest any surface node lies inside the scan by its exact signed distance, with the obstacle as posed at
    /// `time`: over every node if `all`, else over those the grid puts inside the scan or less than the band outside
    /// it. Not finite if a node's position is not.
    fn exact_depth(
        &self,
        obstacle: &Obstacle,
        displacements: &[[f64; 3]],
        time: f64,
        all: bool,
    ) -> f64 {
        let pose = obstacle.pose_at(time);
        let rest = self.model.rest_positions();
        let chunks = std::thread::available_parallelism().map_or(4, std::num::NonZero::get);
        let per = self.surface.len().div_ceil(chunks).max(1);
        std::thread::scope(|scope| {
            let handles: Vec<_> = self
                .surface
                .chunks(per)
                .map(|nodes| {
                    scope.spawn(move || {
                        nodes.iter().try_fold(0.0_f64, |deepest, &n| {
                            let (x, u) = (rest[n as usize], displacements[n as usize]);
                            let body = pose_to_body(pose, [x[0] + u[0], x[1] + u[1], x[2] + u[2]]);
                            if !body.iter().all(|c| c.is_finite()) {
                                return None;
                            }
                            if !all && obstacle.sample(body).distance >= PRODUCT_BAKE.band {
                                return Some(deepest);
                            }
                            Some(deepest.max(-self.scene.distance.signed(Point3::from(body))))
                        })
                    })
                })
                .collect();
            handles
                .into_iter()
                .map(|h| h.join().unwrap())
                .try_fold(0.0, |deepest: f64, chunk| chunk.map(|d| deepest.max(d)))
                .unwrap_or(f64::NAN)
        })
    }
}

/// A run: its reads, its hold's four windows, and how it went.
struct Run {
    spec: Spec,
    reads: Vec<Read>,
    quarters: Vec<Snapshot>,
    clock: Clock,
    steps: u64,
    estimates: u64,
    stopped: Option<RunError>,
    /// The deepest any surface node lies inside the scan at the end, by its exact distance.
    end_depth: f64,
    collapse: Collapse,
    /// The setup and stepping seconds, and the steps, of the attempts `press` made before this run and did not keep
    /// (§16y rule 1, §16x rule 6), so G6 counts them.
    earlier: (f64, u64),
}

/// Step until `end`, reading every new monitor read's exact depth; then read once more if the last step was not read.
fn drive<E: Executor>(
    press: &Press,
    obstacle: &Obstacle,
    stepper: &mut Stepper<E>,
    end: f64,
    (reads, clock, instruments, collapse): (&mut Vec<Read>, &mut Clock, bool, &mut Collapse),
) -> Result<(), RunError> {
    let mut seen = stepper.samples().len();
    let mut record = |stepper: &mut Stepper<E>, reads: &mut Vec<Read>, clock: &mut Clock| {
        if stepper.samples().len() == seen {
            return;
        }
        seen = stepper.samples().len();
        let started = Instant::now();
        let sample = *stepper.samples().last().unwrap();
        // A read that is not finite stops the run; its positions are not read.
        let exact_depth = if instruments && sample.monitors.finite() {
            let displacements = stepper.executor_mut().snapshot().displacements;
            let outputs = stepper.executor_mut().phase_outputs();
            for (e, ratio) in against_nodes(press.model, &outputs).into_iter().enumerate() {
                collapse.least = collapse.least.min(ratio);
                if ratio < 0.5 {
                    collapse.elements.insert(e);
                }
            }
            press.exact_depth(obstacle, &displacements, sample.time, false)
        } else {
            f64::NAN
        };
        reads.push(Read {
            sample,
            travel: press.travel(sample.time),
            exact_depth,
        });
        clock.instruments += started.elapsed().as_secs_f64();
    };
    while stepper.time() < end {
        let started = Instant::now();
        let result = stepper.step();
        clock.stepping += started.elapsed().as_secs_f64();
        record(stepper, reads, clock);
        result?;
    }
    if stepper
        .samples()
        .last()
        .is_none_or(|s| s.step != stepper.steps())
    {
        let started = Instant::now();
        let result = stepper.run_until(end);
        clock.stepping += started.elapsed().as_secs_f64();
        record(stepper, reads, clock);
        result?;
    }
    Ok(())
}

/// A stepper over `press` from time 0, reading often enough that a read reaches at most [`READ_TRAVEL`] at the top
/// speed; and the clock with its setup.
fn begin<E: Executor>(
    press: &Press,
    obstacle: &Obstacle,
    make: impl FnOnce(&ExplicitModel, &Obstacle) -> E,
    damping: f64,
    reestimate_every: u64,
) -> (Stepper<E>, Clock) {
    let rest = rest_step(press.model, obstacle);
    let started = Instant::now();
    let executor = make(press.model, obstacle);
    let top_speed = press.start / (0.9 * press.loading);
    let monitor_every = ((READ_TRAVEL / (top_speed * rest)).floor() as u64).max(1);
    let config = StepperConfig {
        monitor_every,
        reestimate_every,
        ..StepperConfig::new(damping)
    };
    let stepper = Stepper::new(executor, config, 0.0);
    let clock = Clock {
        setup: started.elapsed().as_secs_f64(),
        ..Clock::default()
    };
    (stepper, clock)
}

/// Run `press` from the start through the hold, reading the hold in four windows.
fn run<E: Executor>(
    press: &Press,
    obstacle: &Obstacle,
    make: impl FnOnce(&ExplicitModel, &Obstacle) -> E,
    damping: f64,
    spec: Spec,
) -> (Run, Stepper<E>) {
    let (mut stepper, mut clock) = begin(press, obstacle, make, damping, spec.reestimate_every);
    let mut reads = Vec::new();
    let mut quarters = Vec::new();
    let mut collapse = Collapse::default();
    let mut stopped = drive(
        press,
        obstacle,
        &mut stepper,
        spec.loading,
        (&mut reads, &mut clock, spec.instruments, &mut collapse),
    )
    .err();
    for quarter in 1..=4 {
        if stopped.is_some() {
            break;
        }
        stepper.open_window();
        stopped = drive(
            press,
            obstacle,
            &mut stepper,
            spec.loading + spec.hold * f64::from(quarter) / 4.0,
            (&mut reads, &mut clock, spec.instruments, &mut collapse),
        )
        .err();
        stepper.close_window();
        let started = Instant::now();
        quarters.push(stepper.executor_mut().snapshot());
        clock.instruments += started.elapsed().as_secs_f64();
    }
    let end_depth = quarters
        .last()
        .filter(|_| stopped.is_none())
        .map_or(f64::NAN, |last| {
            press.exact_depth(obstacle, &last.displacements, stepper.time(), true)
        });
    let run = Run {
        spec,
        reads,
        quarters,
        clock,
        steps: stepper.steps(),
        estimates: stepper.estimates(),
        stopped,
        end_depth,
        collapse,
        earlier: (0.0, 0),
    };
    (run, stepper)
}

/// Two windows' sums as one window.
fn joined(a: &Snapshot, b: &Snapshot) -> Snapshot {
    let add3 = |x: &[[f64; 3]], y: &[[f64; 3]]| -> Vec<[f64; 3]> {
        x.iter()
            .zip(y)
            .map(|(p, q)| [p[0] + q[0], p[1] + q[1], p[2] + q[2]])
            .collect()
    };
    Snapshot {
        displacement_sums: add3(&a.displacement_sums, &b.displacement_sums),
        normal_force_sums: a
            .normal_force_sums
            .iter()
            .zip(&b.normal_force_sums)
            .map(|(p, q)| p + q)
            .collect(),
        friction_sums: add3(&a.friction_sums, &b.friction_sums),
        accumulated_steps: a.accumulated_steps + b.accumulated_steps,
        ..b.clone()
    }
}

/// D1's readings from a run, and what is printed beside them.
#[derive(Clone, Copy, Debug, Default)]
struct Readings {
    /// The largest mean push over [`PEAK_WINDOW`], and over [`PEAK_BESIDE`].
    peak: f64,
    beside: [f64; 2],
    /// The largest mean push over [`PUSH_TRAVEL`], and the travel at its window's centre.
    share: f64,
    share_centre: f64,
    /// The most-loaded 1 cm² patch over the hold's last two quarters, and each quarter's.
    patch: f64,
    quarters: [f64; 4],
    p95: f64,
    pointwise_peak: f64,
    loaded_area: f64,
    /// The patch centre's arc along the centreline, over its length.
    patch_arc: f64,
}

impl Readings {
    /// The readings of a run that is not valid: none.
    const fn not_read() -> Self {
        Self {
            peak: f64::NAN,
            beside: [f64::NAN; 2],
            share: f64::NAN,
            share_centre: f64::NAN,
            patch: f64::NAN,
            quarters: [f64::NAN; 4],
            p95: f64::NAN,
            pointwise_peak: f64::NAN,
            loaded_area: f64::NAN,
            patch_arc: f64::NAN,
        }
    }
}

/// The largest mean over `window` of `(travel, work)` samples' work per unit travel, and the travel at that window's
/// centre; the work linear in the travel between samples.
fn best_window(samples: &[(f64, f64)], window: f64) -> Option<(f64, f64)> {
    let work_at = |x: f64| {
        let after = samples
            .partition_point(|s| s.0 < x)
            .clamp(1, samples.len() - 1);
        let ((x0, w0), (x1, w1)) = (samples[after - 1], samples[after]);
        if x1 > x0 {
            w0 + (w1 - w0) * (x - x0) / (x1 - x0)
        } else {
            w1
        }
    };
    let (first, last) = (samples.first()?.0, samples.last()?.0);
    samples
        .iter()
        .flat_map(|s| [s.0, s.0 + window])
        .filter(|&x| x >= first + window && x <= last)
        .map(|x| {
            (
                (work_at(x) - work_at(x - window)) / window,
                x - 0.5 * window,
            )
        })
        .reduce(|a, b| if b.0 > a.0 { b } else { a })
}

impl Run {
    fn samples(&self) -> Vec<Sample> {
        self.reads.iter().map(|r| r.sample).collect()
    }

    fn last(&self) -> Monitors {
        self.reads
            .last()
            .map_or_else(Monitors::default, |r| r.sample.monitors)
    }

    /// `(travel, work)` from the start through the loading.
    fn work(&self) -> Vec<(f64, f64)> {
        std::iter::once((0.0, 0.0))
            .chain(
                self.reads
                    .iter()
                    .filter(|r| r.sample.time <= self.spec.loading + 1e-12)
                    .map(|r| (r.travel, r.sample.monitors.obstacle_work)),
            )
            .collect()
    }

    fn readings(&self, press: &Press, obstacle: &Obstacle) -> Readings {
        let work = self.work();
        let peak_over = |window: f64| work_peak(&work, window).unwrap_or(f64::NAN);
        let (share, share_centre) = best_window(&work, PUSH_TRAVEL).unwrap_or((f64::NAN, f64::NAN));
        let mut readings = Readings {
            peak: peak_over(PEAK_WINDOW),
            beside: PEAK_BESIDE.map(peak_over),
            share,
            share_centre,
            ..Readings::default()
        };
        if self.quarters.len() < 4 {
            return readings;
        }
        let seat = self.spec.loading + self.spec.hold;
        let contact = |window: &Snapshot| WindowContact::read(press.model, window, obstacle, seat);
        for (q, window) in self.quarters.iter().enumerate() {
            readings.quarters[q] = contact(window)
                .patch_peak(PROBE_AREA)
                .map_or(0.0, |p| p.pressure);
        }
        let seated = contact(&joined(&self.quarters[2], &self.quarters[3]));
        if let Some(patch) = seated.patch_peak(PROBE_AREA) {
            readings.patch = patch.pressure;
            readings.loaded_area = seated.loaded_area_at(PROBE_AREA, patch.centre);
            readings.patch_arc = press
                .scene
                .path
                .centreline()
                .arc_of(Point3::from(patch.centre))
                / press.scene.length();
        }
        let pressures = seated.pressures();
        readings.p95 = area_percentile(&pressures, 0.05).unwrap_or(f64::NAN);
        readings.pointwise_peak = pressures.iter().map(|&(p, _)| p).fold(f64::NAN, f64::max);
        readings
    }

    /// The sideways force and twist: at the read of the largest per-read push over the loading, `|F⊥|` over that push
    /// and over `Σ f_n`; the largest `|F⊥| / Σ f_n` over the loading; and at the seat (the hold's last half) `|F⊥| /
    /// Σ f_n` and `|twist| / (Σ f_n ℓ)`, ℓ the centreline's length.
    fn across(&self, press: &Press, obstacle: &Obstacle) -> [f64; 5] {
        let tip = press.scene.tip;
        let interval = |time: f64| {
            let last = obstacle.poses.len() - 1;
            let k = ((time / obstacle.interval).floor() as usize).min(last - 1);
            (obstacle.poses[k], obstacle.poses[k + 1])
        };
        let mut worst = 0.0_f64;
        let mut at_peak = (f64::NEG_INFINITY, 0.0, 0.0);
        let mut previous = (0.0, 0.0);
        for read in &self.reads {
            let m = read.sample.monitors;
            let (from, to) = interval(read.sample.time.min(self.spec.loading));
            let Some(along) = along_path(from, to, tip) else {
                continue;
            };
            let across = norm(sideways(m.contact_force, along));
            if read.sample.time <= self.spec.loading && m.normal_force > 0.0 {
                worst = worst.max(across / m.normal_force);
                let advance = read.travel - previous.0;
                if advance > 0.0 {
                    let push = (m.obstacle_work - previous.1) / advance;
                    if push > at_peak.0 {
                        at_peak = (push, across / push, across / m.normal_force);
                    }
                }
            }
            previous = (read.travel, m.obstacle_work);
        }
        let seat = self.spec.loading + 0.5 * self.spec.hold;
        let held: Vec<&Read> = self.reads.iter().filter(|r| r.sample.time > seat).collect();
        let weight: f64 = held.iter().map(|r| r.sample.monitors.steps as f64).sum();
        let mean = |f: &dyn Fn(&Monitors) -> [f64; 3]| {
            held.iter().fold([0.0; 3], |sum, r| {
                let v = f(&r.sample.monitors);
                let w = r.sample.monitors.steps as f64 / weight;
                [sum[0] + w * v[0], sum[1] + w * v[1], sum[2] + w * v[2]]
            })
        };
        let force = mean(&|m| m.contact_force);
        let moment = mean(&|m| m.contact_moment);
        let normal: f64 = held
            .iter()
            .map(|r| r.sample.monitors.normal_force * r.sample.monitors.steps as f64 / weight)
            .sum();
        let (from, to) = interval(self.spec.loading);
        let along = along_path(from, to, tip).unwrap_or([0.0; 3]);
        let pose = obstacle.pose_at(self.spec.loading + self.spec.hold);
        let twist = moment_about(moment, force, pose, tip);
        [
            at_peak.1,
            at_peak.2,
            worst,
            norm(sideways(force, along)) / normal,
            norm(twist) / (normal * press.scene.length()),
        ]
    }

    /// The contact work over each quarter of the hold, over the internal energy at the seat.
    fn hold_work(&self) -> [f64; 4] {
        let at = |time: f64| {
            self.reads
                .iter()
                .min_by(|a, b| {
                    (a.sample.time - time)
                        .abs()
                        .total_cmp(&(b.sample.time - time).abs())
                })
                .map_or(f64::NAN, |r| r.sample.monitors.contact_work)
        };
        let seated = self.last().internal_energy;
        let edges: Vec<f64> = (0..=4)
            .map(|q| at(self.spec.loading + self.spec.hold * f64::from(q) / 4.0))
            .collect();
        [0, 1, 2, 3].map(|q| (edges[q + 1] - edges[q]) / seated)
    }
}

fn norm(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// A change `b / a − 1`.
fn change(a: f64, b: f64) -> f64 {
    b / a - 1.0
}

/// The label of a run's corner.
fn corner(spec: &Spec) -> String {
    format!(
        "μ_f {:.3}, {}, loading ×{:.0}",
        spec.friction,
        match (spec.gpu, spec.wide) {
            (true, _) => "GPU f32",
            (false, true) => "f64",
            (false, false) => "f32",
        },
        spec.loading
    )
}

/// Print a run's line: its readings, the prints of rules 6–9, and its cost. LOCAL figures and PUBLIC ratios apart.
fn report(label: &str, run: &Run, readings: &Readings, press: &Press, obstacle: &Obstacle) {
    let last = run.last();
    let samples = run.samples();
    let first_contact = run
        .reads
        .iter()
        .find(|r| r.sample.monitors.normal_force > 0.0)
        .map_or(0.0, |r| r.sample.time);
    let window = run.spec.loading + 0.5 * run.spec.hold;
    let fmt = |x: Option<f64>| x.map_or_else(|| "n/a".to_owned(), |v| format!("{:.2}%", 100.0 * v));
    let bar = g2_bar(press.scene.inset());
    let exact = run.reads.iter().map(|r| r.exact_depth).fold(0.0, f64::max);
    let across = run.across(press, obstacle);
    let hold = run.hold_work();
    let settled = (0..4)
        .find(|&q| {
            (q..4).all(|k| change(readings.quarters[3], readings.quarters[k]).abs() <= SETTLE_BAR)
        })
        .unwrap_or(4);
    if let Some(RunError::NonFinite { time, .. }) = run.stopped {
        println!(
            "  {label}: ⚠ stopped, not finite, at {:.3} of the loading time, the walk at {:.3} of its start [PUBLIC]",
            time / run.spec.loading,
            1.0 - press.travel(time) / press.start
        );
    }
    println!(
        "  {label}: stopped {:?}; steps {}, estimates {}; setup {:.1} s, stepping {:.1} s, instruments {:.1} s \
         [LOCAL]",
        run.stopped,
        run.steps,
        run.estimates,
        run.clock.setup,
        run.clock.stepping,
        run.clock.instruments
    );
    if run.spec.instruments {
        println!(
            "    the step re-estimated every {} steps; the least element J over its nodes' averaged J {:.3} [PUBLIC]; \
             {} elements under half at some read [LOCAL]",
            run.spec.reestimate_every,
            run.collapse.least,
            run.collapse.elements.len()
        );
    } else {
        println!(
            "    the step re-estimated every {} steps; the collapse not read, the instruments off [PUBLIC]",
            run.spec.reestimate_every
        );
    }
    println!(
        "    D1 [LOCAL]: peak push {:.4} N (0.5 mm {:.4}, 2 mm {:.4}); 10 mm push {:.4} N; patch {:.3} kPa; p95 {:.3} \
         kPa; pointwise peak {:.3} kPa",
        readings.peak,
        readings.beside[0],
        readings.beside[1],
        readings.share,
        1e-3 * readings.patch,
        1e-3 * readings.p95,
        1e-3 * readings.pointwise_peak
    );
    println!(
        "    [PUBLIC] loaded area in the patch / 1 cm² {:.3}; patch centre at {:.2} of the centreline from the seated \
         tip; 0.5 and 2 mm peaks over 1 mm's {:.3} {:.3}",
        readings.loaded_area / PROBE_AREA,
        readings.patch_arc,
        readings.beside[0] / readings.peak,
        readings.beside[1] / readings.peak
    );
    println!(
        "    [PUBLIC] across: at the largest read push, |F⊥|/push {:.3}, |F⊥|/Σf_n {:.3}; largest |F⊥|/Σf_n over the \
         loading {:.3}; seated |F⊥|/Σf_n {:.3}, |twist|/(Σf_n ℓ) {:.4}",
        across[0], across[1], across[2], across[3], across[4]
    );
    println!(
        "    [PUBLIC] G2 exact, deepest over the reads / bar {:.3}, at the end over every surface node / bar {:.3} (G1 \
         the same); grid every step / bar {:.3}; deepest predicted / band {:.3}{}; coarse corrections {}",
        exact / bar,
        run.end_depth / bar,
        last.max_penetration / bar,
        last.deepest_prediction / PRODUCT_BAKE.band,
        if last.deepest_prediction > 0.5 * PRODUCT_BAKE.band {
            " ⚠ past half the band"
        } else {
            ""
        },
        last.coarse_corrections
    );
    println!(
        "    [PUBLIC] gates: KE/IE over the loading from first contact {}, over the window {}; balance {}; inverted {}; \
         quarters' patch over the last's {:.4} {:.4} {:.4} 1, settled from quarter {}{}; contact work per quarter / \
         IE {:.2e} {:.2e} {:.2e} {:.2e}{}",
        fmt(gates::kinetic_over_internal(
            &samples,
            first_contact,
            run.spec.loading
        )),
        fmt(gates::kinetic_over_internal(
            &samples,
            window,
            run.spec.loading + run.spec.hold
        )),
        fmt(gates::energy_balance(&samples)),
        gates::inverted(&samples),
        readings.quarters[0] / readings.quarters[3],
        readings.quarters[1] / readings.quarters[3],
        readings.quarters[2] / readings.quarters[3],
        settled + 1,
        if change(readings.quarters[2], readings.quarters[3]).abs() > SETTLE_BAR {
            " ⚠ the last two differ past rule 6's 1 %"
        } else {
            ""
        },
        hold[0],
        hold[1],
        hold[2],
        hold[3],
        if hold[2] + hold[3] > PUMPING_BAR {
            " ⚠ past rule 9's bar"
        } else {
            ""
        }
    );
}

/// What a stage builds before its runs: the scene, the bake on the sampled path, and h_K2.
struct Stage {
    scene: Scene,
    obstacle: Obstacle,
    /// The sampled path's count of intervals, which the loading time does not change.
    intervals: f64,
    h_k2: f64,
}

impl Stage {
    fn new(title: &str) -> Self {
        println!("\n══ step 7's first run on base_mold: {title} ══");
        let scene = Scene::load();
        let (h_k2, _) = h_k2();
        let join = scene.path.join().expect("the scan leaves the device");
        println!("path: join / length {:.2} [PUBLIC]", join / scene.length());
        let started = Instant::now();
        let baked = bake_surface(&scene.distance, PRODUCT_BAKE).unwrap();
        let bake = started.elapsed().as_secs_f64();
        println!(
            "bake: {bake:.1} s [LOCAL]; {:.2} of D4, once per scan and band [PUBLIC]",
            bake / D4_SECONDS
        );
        // Sampled once at a nominal loading from the join: the poses depend on neither; each run sets its interval.
        let sampled = scene.path.sampled(join, 1.0, SAMPLING_BAR).unwrap();
        let intervals = (sampled.poses.len() - 1) as f64;
        let obstacle = sampled.obstacle(baked, 0.0);
        Self {
            scene,
            obstacle,
            intervals,
            h_k2,
        }
    }

    /// A wall's line; returns its start, which must be the join, where the sampled path starts.
    fn wall_line(&self, label: &str, wall: &Wall, model: &ExplicitModel) -> f64 {
        let start = self
            .scene
            .path
            .start(
                &boundary_points(model),
                &self.scene.distance,
                START_CLEARANCE,
            )
            .unwrap();
        let share = wall.mount.len() as f64 / wall.skin as f64;
        println!(
            "wall {label}: {} elements, lattice {:.3} mm, start walk {:.2} mm, mount {} of {} skin vertices [LOCAL]; \
             h / h_K2 {:.3}, one piece {}, start at the join {}, the travel longer than the centreline and 5 mm {}, \
             the mount under {:.1} of the skin's vertices [PUBLIC]",
            model.element_count(),
            1e3 * wall.cell,
            1e3 * start,
            wall.mount.len(),
            wall.skin,
            element_size(model) / self.h_k2,
            pieces(model) == 1,
            Some(start) == self.scene.path.join(),
            start > self.scene.length() + 0.005,
            (share * 10.0).ceil() / 10.0
        );
        assert_eq!(
            Some(start),
            self.scene.path.join(),
            "the sampled path starts at the join; a wall that starts elsewhere needs its own"
        );
        start
    }

    /// The budget's loading time from walk `start`: the tube's rung speed in the wall's innermost material (§16r).
    fn budget_loading(wall: &Wall, start: f64) -> f64 {
        start / (0.9 * speed_over_shear_wave() * wall.shear_speed())
    }

    /// ξ 0.05 at the product's axial-shear period by the tube's formula, `4ℓ/c_s` (§16x).
    fn damping(&self, wall: &Wall) -> f64 {
        2.0 * 0.05 * TAU / (4.0 * self.scene.length() / wall.shear_speed())
    }

    /// Pose the obstacle for `spec`.
    fn set(&mut self, spec: &Spec) {
        self.obstacle.friction = spec.friction;
        self.obstacle.start = 0.0;
        self.obstacle.interval = spec.loading / self.intervals;
    }
}

/// One run, printed.
fn press(
    stage: &mut Stage,
    wall: &Wall,
    model: &ExplicitModel,
    start: f64,
    spec: Spec,
    label: &str,
) -> (Run, Readings) {
    stage.set(&spec);
    let press = Press {
        scene: &stage.scene,
        model,
        surface: surface_nodes(model),
        start,
        loading: spec.loading,
    };
    let damping = stage.damping(wall);
    let (run, readings) = if spec.gpu {
        let ctx = GpuContext::new().expect("a GPU adapter");
        let (run, _) = run(
            &press,
            &stage.obstacle,
            |m, o| GpuExecutor::new(&ctx, m, o).unwrap(),
            damping,
            spec,
        );
        let readings = run.readings(&press, &stage.obstacle);
        (run, readings)
    } else if spec.wide {
        let (run, _) = run(
            &press,
            &stage.obstacle,
            |m, o| cpu::f64::CpuExecutor::new(m, o).unwrap(),
            damping,
            spec,
        );
        let readings = run.readings(&press, &stage.obstacle);
        (run, readings)
    } else {
        let (run, _) = run(
            &press,
            &stage.obstacle,
            |m, o| cpu::f32::CpuExecutor::new(m, o).unwrap(),
            damping,
            spec,
        );
        let readings = run.readings(&press, &stage.obstacle);
        (run, readings)
    };
    report(label, &run, &readings, &press, &stage.obstacle);
    // A run whose readings do not stand reads nothing for the rules; its readings are printed above. A hold that did
    // not settle is run again with a 0.4 s hold (rule 6).
    match falls_short(&run, &readings) {
        None => (run, readings),
        // §16y rule 1: a run that went non-finite, failed a validity gate, or inverted an element with the loop's
        // re-estimate is run again with the step re-estimated every 50 steps.
        Some(reason)
            if ["not finite", "validity gate", "K4"]
                .iter()
                .any(|r| reason.contains(r))
                && spec.reestimate_every > 50 =>
        {
            println!(
                "    ⚠ {reason}: run again by §16y rule 1, the step re-estimated every 50 steps [PUBLIC]"
            );
            let again = self::press(
                stage,
                wall,
                model,
                start,
                Spec {
                    reestimate_every: 50,
                    ..spec
                },
                &format!("{label}, re-estimated every 50 steps"),
            );
            counting(run, again)
        }
        Some(reason) if reason.contains("rule 6") && spec.hold < 0.4 => {
            println!("    ⚠ {reason}: run again with a 0.4 s hold [PUBLIC]");
            let again = self::press(
                stage,
                wall,
                model,
                start,
                Spec { hold: 0.4, ..spec },
                &format!("{label}, 0.4 s hold"),
            );
            counting(run, again)
        }
        Some(reason) => {
            println!("    ⚠ {reason}: its readings take no part in the rules [PUBLIC]");
            (run, Readings::not_read())
        }
    }
}

/// A re-run's result, carrying the attempt it replaced in its `earlier` cost.
fn counting(attempt: Run, (mut again, readings): (Run, Readings)) -> (Run, Readings) {
    again.earlier.0 += attempt.earlier.0 + attempt.clock.setup + attempt.clock.stepping;
    again.earlier.1 += attempt.earlier.1 + attempt.steps;
    (again, readings)
}

/// What a run presses: the wall, its model and its start.
#[derive(Clone, Copy)]
struct Pressed<'a> {
    wall: &'a Wall,
    model: &'a ExplicitModel,
    start: f64,
}

/// A wall mounted for a stage's runs: its model as it is, its start and its loading.
struct Mounted {
    wall: Wall,
    model: ExplicitModel,
    start: f64,
    loading: f64,
}

impl Mounted {
    /// `wall` lowered at ν 0.49 with Ecoflex 00-30's `η/μ`, its line printed, at `factor` times the budget's loading.
    fn new(stage: &Stage, wall: Wall, label: &str, factor: f64) -> Self {
        let model = wall.model(POISSON, 1.0, 1.0);
        let start = stage.wall_line(label, &wall, &model);
        let loading = factor * Stage::budget_loading(&wall, start);
        Self {
            wall,
            model,
            start,
            loading,
        }
    }

    fn on(&self) -> Pressed<'_> {
        Pressed {
            wall: &self.wall,
            model: &self.model,
            start: self.start,
        }
    }
}

/// A run's readings: the settings of the run that was kept, its label, the labels of its kept runs in which K4 failed
/// (§15a: an element inverted though the validity gates held) and of those that stopped with an element inverted (K4
/// not read), and its readings every 50 steps once a comparison asked for them (§16z).
#[derive(Clone)]
struct Cell {
    spec: Spec,
    label: String,
    k4: Vec<String>,
    k4_unread: Vec<String>,
    readings: Readings,
    fifty: Option<Readings>,
}

impl Cell {
    /// Press `on` at `spec`, by [`press`], with §16y rule 1's re-run.
    fn press(stage: &mut Stage, on: Pressed, spec: Spec, label: String) -> (Self, Run) {
        let (run, readings) = press(stage, on.wall, on.model, on.start, spec, &label);
        let (stopped, inverted) = (run.stopped.is_some(), gates::inverted(&run.samples()));
        let named = |yes: bool| if yes { vec![label.clone()] } else { Vec::new() };
        let (k4, k4_unread) = (
            named(k4_fails(stopped, gates_hold(&run), inverted)),
            named(stopped && inverted),
        );
        let cell = Self {
            spec: run.spec,
            label,
            k4,
            k4_unread,
            readings,
            fifty: None,
        };
        (cell, run)
    }

    /// The re-estimate interval its kept run stood at, or ended at.
    fn every(&self) -> u64 {
        self.spec.reestimate_every
    }

    /// Whether its readings stand ([`falls_short`]).
    fn stood(&self) -> bool {
        self.readings.patch.is_finite()
    }

    /// Its readings from its run made again with the step re-estimated every 50 steps, once ([`run_again`] asks only of
    /// a run kept at another interval). Where both stand, the change is printed: the interval's own effect.
    fn at_fifty(&mut self, stage: &mut Stage, on: Pressed) -> Readings {
        if let Some(readings) = self.fifty {
            return readings;
        }
        let spec = Spec {
            reestimate_every: 50,
            ..self.spec
        };
        let label = format!("{}, every 50 steps to match its comparison", self.label);
        let (again, _) = Self::press(stage, on, spec, label);
        let readings = again.readings;
        let stood = again.stood();
        self.k4.extend(again.k4);
        self.k4_unread.extend(again.k4_unread);
        if stood {
            println!(
                "    the interval: every 50 steps over every {}, on a run that stood at both: patch {:+.3} %, 10 mm \
                 push {:+.3} %, peak push {:+.3} % [PUBLIC]",
                self.every(),
                100.0 * change(self.readings.patch, readings.patch),
                100.0 * change(self.readings.share, readings.share),
                100.0 * change(self.readings.peak, readings.peak)
            );
        } else {
            println!("    the interval: the run made again every 50 steps did not stand [PUBLIC]");
        }
        self.fifty = Some(readings);
        readings
    }
}

/// Whether K4 failed in a kept run: it did not stop, its validity gates held, and an element inverted (§15a).
fn k4_fails(stopped: bool, gates: bool, inverted: bool) -> bool {
    !stopped && gates && inverted
}

/// Whether two runs need one to run again every 50 steps to be read at one interval (§16z): both stood, at different
/// intervals.
fn needs_one_interval(a: &Cell, b: &Cell) -> bool {
    a.every() != b.every() && a.stood() && b.stood()
}

/// Two runs' readings at one re-estimate interval (§16z): where §16y rule 1 ran one every 50 steps and the other stood at
/// the loop's interval, the other is run again every 50 steps. A run that did not stand is read as it is, which the
/// rules read as not judged.
fn at_one_interval(
    stage: &mut Stage,
    (a, on_a): (&mut Cell, Pressed),
    (b, on_b): (&mut Cell, Pressed),
) -> (Readings, Readings) {
    let [again_a, again_b] = run_again(a, b);
    let read_a = if again_a {
        a.at_fifty(stage, on_a)
    } else {
        a.readings
    };
    let read_b = if again_b {
        b.at_fifty(stage, on_b)
    } else {
        b.readings
    };
    (read_a, read_b)
}

/// Which of two runs is run again every 50 steps to read them at one interval: the one kept at another interval, when
/// both stood at different intervals ([`needs_one_interval`]).
fn run_again(a: &Cell, b: &Cell) -> [bool; 2] {
    let needs = needs_one_interval(a, b);
    [needs && a.every() != 50, needs && b.every() != 50]
}

/// The K4 line over a stage's cells, the element as it is.
fn k4_line(cells: &[&Cell]) {
    println!("{}", k4_text(cells));
}

/// K4 over a stage's cells: every kept run in which it failed, and every kept run that stopped with an element
/// inverted, where it is not read (§16x).
fn k4_text(cells: &[&Cell]) -> String {
    let named = |f: fn(&Cell) -> &Vec<String>| {
        cells
            .iter()
            .flat_map(|c| f(c))
            .map(String::as_str)
            .collect::<Vec<_>>()
            .join("; ")
    };
    let (failed, unread) = (named(|c| &c.k4), named(|c| &c.k4_unread));
    let mut text = if failed.is_empty() {
        "K4 [PUBLIC]: holds in every kept run that stood (an inversion §16y rule 1 ran again is not K4's, §15a)"
            .to_owned()
    } else {
        format!(
            "K4 [PUBLIC]: ⚠ FAILS, an element inverted in a valid run: {failed}; §15a sends K4 to the element or the \
             loading time, and it is one of §15g step 2's stop criteria"
        )
    };
    if !unread.is_empty() {
        text.push_str(&format!(
            "; not read in the kept runs that stopped with an element inverted: {unread}"
        ));
    }
    text
}

/// The interval rule's reading over a stage's cells.
fn interval_line(cells: &[&Cell]) {
    println!("{}", interval_text(cells));
}

/// The interval rule's reading: of the runs made again every 50 steps that stood at both intervals, the most a
/// deciding reading moved (the patch, and the geometric share at μ_f 0), against K3's 0.5 %.
fn interval_text(cells: &[&Cell]) -> String {
    let moved: Vec<f64> = cells
        .iter()
        .filter_map(|cell| {
            let other = cell.fifty.filter(|r| r.patch.is_finite())?;
            let share = if cell.spec.friction == 0.0 {
                change(cell.readings.share, other.share).abs()
            } else {
                0.0
            };
            Some(change(cell.readings.patch, other.patch).abs().max(share))
        })
        .collect();
    let Some(largest) = moved.iter().copied().reduce(f64::max) else {
        return "§16z's interval rule [PUBLIC]: no run made again every 50 steps stood at both, so not read"
            .to_owned();
    };
    format!(
        "§16z's interval rule [PUBLIC]: {} runs made again every 50 steps stood at both; the most a deciding reading \
         moved {:.3} %{}",
        moved.len(),
        100.0 * largest,
        if largest > K3_BAR {
            " ⚠ past K3's 0.5 %: D1's readings depend on the step control, which goes to Jon beside D1's size"
        } else {
            ""
        }
    )
}

/// The replicate a doubling from `SIZES[k]` is read against, as an index into `replicates` (indices into [`SIZES`]):
/// the one at its coarser size, or the nearest coarser.
fn scatter_for(k: usize, replicates: &[usize]) -> usize {
    replicates
        .iter()
        .rposition(|&r| r <= k)
        .expect("a replicate at or below every size")
}

/// The model's boundary nodes.
fn surface_nodes(model: &ExplicitModel) -> Vec<u32> {
    let mut nodes: Vec<u32> = model
        .surface_triangles()
        .iter()
        .flatten()
        .copied()
        .collect();
    nodes.sort_unstable();
    nodes.dedup();
    nodes
}

fn env_number(name: &str, default: f64) -> f64 {
    std::env::var(name).map_or(default, |v| {
        v.parse()
            .expect("the stage's environment variable is a number")
    })
}

/// The first run's pairing's corners: μ_f 0, its lowest and its highest.
fn corners() -> [f64; 3] {
    PAIRINGS
        .iter()
        .find(|p| p.name == PAIRING)
        .expect("the first run's pairing")
        .corners()
}

fn isometry(pose: Pose) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(pose.tx, pose.ty, pose.tz),
        UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            pose.qw, pose.qx, pose.qy, pose.qz,
        )),
    )
}

fn pose_of(isometry: Isometry3<f64>) -> Pose {
    let (q, t) = (isometry.rotation, isometry.translation.vector);
    Pose {
        qw: q.w,
        qx: q.i,
        qy: q.j,
        qz: q.k,
        tx: t.x,
        ty: t.y,
        tz: t.z,
    }
}

/// Rule 10's four degrees of freedom about a pose: two moves across the path, and two turns about axes across it
/// through the posed seated tip. The turn about the path's own direction is left out.
struct Freedom {
    base: Isometry3<f64>,
    /// The seated tip, in the scan's frame and posed.
    tip: [f64; 3],
    posed_tip: Point3<f64>,
    along: Vector3<f64>,
    axes: [Vector3<f64>; 2],
    /// The probe steps: [`PROBE_STEP`] for the moves, and for the turns the angle that moves the scan's farthest point
    /// inside the device by it.
    steps: [f64; 4],
}

impl Freedom {
    fn new(scene: &Scene, base: Pose, along: [f64; 3]) -> Self {
        let base = isometry(base);
        let along = Vector3::from(along).normalize();
        let least = (0..3)
            .min_by(|&a, &b| along[a].abs().total_cmp(&along[b].abs()))
            .unwrap();
        let first = along.cross(&Vector3::ith(least, 1.0)).normalize();
        let second = along.cross(&first);
        let posed_tip = base * Point3::from(scene.tip);
        let reach = scene
            .scan
            .vertices
            .iter()
            .map(|v| base * v)
            .filter(|&v| scene.path.inside(v))
            .map(|v| (v - posed_tip).norm())
            .fold(0.0, f64::max);
        let turn = PROBE_STEP / reach;
        Self {
            base,
            tip: scene.tip,
            posed_tip,
            along,
            axes: [first, second],
            steps: [PROBE_STEP, PROBE_STEP, turn, turn],
        }
    }

    fn pose(&self, q: &Vector4<f64>) -> Pose {
        let turn = UnitQuaternion::from_scaled_axis(self.axes[0] * q[2] + self.axes[1] * q[3]);
        let origin = self.posed_tip.coords
            + turn * (self.base.translation.vector - self.posed_tip.coords)
            + self.axes[0] * q[0]
            + self.axes[1] * q[1];
        pose_of(Isometry3::from_parts(
            Translation3::from(origin),
            turn * self.base.rotation,
        ))
    }

    /// The force and the twist about the posed tip on the scan, across the path.
    fn residual(&self, force: [f64; 3], moment: [f64; 3], q: &Vector4<f64>) -> Vector4<f64> {
        let twist = Vector3::from(moment_about(moment, force, self.pose(q), self.tip));
        let force = Vector3::from(force);
        -Vector4::new(
            force.dot(&self.axes[0]),
            force.dot(&self.axes[1]),
            twist.dot(&self.axes[0]),
            twist.dot(&self.axes[1]),
        )
    }

    /// The twist's part about the path's own direction, left out of the freedom.
    fn roll(&self, force: [f64; 3], moment: [f64; 3], q: &Vector4<f64>) -> f64 {
        Vector3::from(moment_about(moment, force, self.pose(q), self.tip)).dot(&self.along)
    }
}

/// A probe's read: the mean resultant and moment over its read window, their change between the window's halves
/// over their size, and the window's sums.
struct Held {
    force: [f64; 3],
    moment: [f64; 3],
    noise: f64,
    window: Snapshot,
    time: f64,
}

/// Move the scan along `poses` over [`MOVE_TIME`], hold [`PROBE_HOLD`], and read [`PROBE_READ`].
fn probe<E: Executor>(
    press: &Press,
    obstacle: &Obstacle,
    stepper: &mut Stepper<E>,
    poses: &[Pose],
    (reads, clock): (&mut Vec<Read>, &mut Clock),
) -> Held {
    let start = stepper.time();
    stepper
        .executor_mut()
        .set_poses(start, MOVE_TIME / (poses.len() - 1) as f64, poses)
        .unwrap();
    drive(
        press,
        obstacle,
        stepper,
        start + MOVE_TIME + PROBE_HOLD,
        (reads, clock, false, &mut Collapse::default()),
    )
    .unwrap();
    let first = stepper.samples().len();
    stepper.open_window();
    let end = start + MOVE_TIME + PROBE_HOLD + PROBE_READ;
    drive(
        press,
        obstacle,
        stepper,
        end,
        (reads, clock, false, &mut Collapse::default()),
    )
    .unwrap();
    stepper.close_window();
    let window = stepper.executor_mut().snapshot();
    let samples = &stepper.samples()[first..];
    let mean = |part: &[Sample], f: &dyn Fn(&Monitors) -> [f64; 3]| {
        let weight: f64 = part.iter().map(|s| s.monitors.steps as f64).sum();
        part.iter().fold([0.0; 3], |sum, s| {
            let (v, w) = (f(&s.monitors), s.monitors.steps as f64 / weight);
            [sum[0] + w * v[0], sum[1] + w * v[1], sum[2] + w * v[2]]
        })
    };
    let half = samples.len() / 2;
    let (early, late) = (
        mean(&samples[..half.max(1)], &|m| m.contact_force),
        mean(&samples[half..], &|m| m.contact_force),
    );
    let force = mean(samples, &|m| m.contact_force);
    Held {
        force,
        moment: mean(samples, &|m| m.contact_moment),
        noise: norm([late[0] - early[0], late[1] - early[1], late[2] - early[2]]) / norm(force),
        window,
        time: end,
    }
}

/// The poses of a probe from `from` to `to` over §15b's ramp, 21 samples.
fn ramp(freedom: &Freedom, from: &Vector4<f64>, to: &Vector4<f64>) -> Vec<Pose> {
    (0..=20)
        .map(|i| freedom.pose(&(from + (to - from) * travelled(f64::from(i) / 20.0, 1.0))))
        .collect()
}

/// Rule 10 from the stepper's current pose: the stiffness across the path by central differences, up to three steps
/// with it toward no sideways force and no twist across the path, and `reading` there and at the start. Returns the
/// start's reading, the end's, and prints the rest.
fn free_scan<E: Executor>(
    press: &Press,
    obstacle: &mut Obstacle,
    stepper: &mut Stepper<E>,
    freedom: &Freedom,
    reading: &dyn Fn(&Held, Pose, &mut Obstacle) -> f64,
    label: &str,
) -> (f64, f64) {
    let (mut reads, mut clock) = (Vec::new(), Clock::default());
    let zero = Vector4::zeros();
    let hold = |stepper: &mut Stepper<E>,
                from: &Vector4<f64>,
                to: &Vector4<f64>,
                reads: &mut Vec<Read>,
                clock: &mut Clock| {
        probe(
            press,
            obstacle,
            stepper,
            &ramp(freedom, from, to),
            (reads, clock),
        )
    };
    let base = hold(stepper, &zero, &zero, &mut reads, &mut clock);
    let start_residual = freedom.residual(base.force, base.moment, &zero);
    let mut noise = base.noise;
    let mut jacobian = Matrix4::zeros();
    let mut at = zero;
    for k in 0..4 {
        let mut columns = [Vector4::zeros(); 2];
        for (side, sign) in [1.0, -1.0].into_iter().enumerate() {
            let q = Vector4::ith(k, sign * freedom.steps[k]);
            let held = hold(stepper, &at, &q, &mut reads, &mut clock);
            noise = noise.max(held.noise);
            columns[side] = freedom.residual(held.force, held.moment, &q);
            at = q;
        }
        jacobian.set_column(k, &((columns[0] - columns[1]) / (2.0 * freedom.steps[k])));
    }
    let inverse = jacobian
        .try_inverse()
        .expect("the stiffness across the path must be invertible");
    let (mut q, mut residual, mut last) = (zero, start_residual, None);
    let size = |r: &Vector4<f64>| (r.fixed_rows::<2>(0).norm(), r.fixed_rows::<2>(2).norm());
    let (force0, twist0) = size(&start_residual);
    for _ in 0..3 {
        let next = q - inverse * residual;
        let held = hold(stepper, &at, &next, &mut reads, &mut clock);
        noise = noise.max(held.noise);
        residual = freedom.residual(held.force, held.moment, &next);
        at = next;
        q = next;
        last = Some(held);
        let (force, twist) = size(&residual);
        if force <= 0.1 * force0 && twist <= 0.1 * twist0 {
            break;
        }
    }
    let last = last.unwrap();
    let (force, twist) = size(&residual);
    let before = reading(&base, freedom.pose(&zero), obstacle);
    let after = reading(&last, freedom.pose(&q), obstacle);
    println!(
        "    rule 10 {label}: moved {:.3} mm across and turned {:.4}° [LOCAL]; residual sideways force {:.3} and twist \
         {:.3} of the fitted pose's, the roll's moment over the twist's {:.3}, the probes' noise {:.4}; the reading \
         changes {:+.2} % ⇒ {} [PUBLIC]",
        1e3 * q.fixed_rows::<2>(0).norm(),
        q.fixed_rows::<2>(2).norm().to_degrees(),
        force / force0,
        twist / twist0,
        freedom.roll(base.force, base.moment, &zero).abs() / twist0,
        noise,
        100.0 * change(before, after),
        if change(before, after).abs() > K5_BAR {
            "recommend to Jon that the contact-guided scan comes forward"
        } else {
            "within 5 %"
        }
    );
    (before, after)
}

/// The patch at a held pose.
fn patch_at_pose(model: &ExplicitModel, held: &Held, pose: Pose, obstacle: &mut Obstacle) -> f64 {
    let saved = (
        std::mem::take(&mut obstacle.poses),
        obstacle.start,
        obstacle.interval,
    );
    obstacle.poses = vec![pose];
    obstacle.start = held.time;
    obstacle.interval = 1.0;
    let patch = WindowContact::read(model, &held.window, obstacle, held.time)
        .patch_peak(PROBE_AREA)
        .map_or(0.0, |p| p.pressure);
    (obstacle.poses, obstacle.start, obstacle.interval) = saved;
    patch
}

/// Why a run's readings do not stand, or `None` if they do (§16x): it stopped, not finite; a validity gate failed
/// (KE/IE at most 5 % over the loading from first contact and over the window, the energy balance at most 1 %; §15a as
/// amended, §16p's ladder); K4 failed, an element inverting in a run the gates call valid (§15a); a correction read the
/// coarse grid (rule 7); or the hold's last two quarters' patches differ by more than 1 % (rule 6).
fn falls_short(run: &Run, readings: &Readings) -> Option<&'static str> {
    if run.stopped.is_some() {
        Some("it stopped, not finite")
    } else if !gates_hold(run) {
        Some("a validity gate failed")
    } else if gates::inverted(&run.samples()) {
        Some("K4 fails: an element inverted in a valid run")
    } else if run.last().coarse_corrections > 0 {
        Some(
            "a correction read the coarse grid: re-bake at twice its deepest predicted point and run again (rule 7)",
        )
    } else if change(readings.quarters[2], readings.quarters[3]).abs() > SETTLE_BAR {
        Some("the hold did not settle (rule 6)")
    } else {
        None
    }
}

/// Whether a run's validity gates hold: KE/IE at most 5 % over the loading from first contact and over the window, and
/// the energy balance at most 1 %.
fn gates_hold(run: &Run) -> bool {
    let samples = run.samples();
    let first_contact = run
        .reads
        .iter()
        .find(|r| r.sample.monitors.normal_force > 0.0)
        .map_or(0.0, |r| r.sample.time);
    let window = run.spec.loading + 0.5 * run.spec.hold;
    let within = |x: Option<f64>, bar: f64| x.is_some_and(|x| x <= bar);
    within(
        gates::kinetic_over_internal(&samples, first_contact, run.spec.loading),
        0.05,
    ) && within(
        gates::kinetic_over_internal(&samples, window, run.spec.loading + run.spec.hold),
        0.05,
    ) && within(gates::energy_balance(&samples), 0.01)
}

fn spec(friction: f64, loading: f64) -> Spec {
    Spec {
        friction,
        loading,
        wide: false,
        gpu: false,
        hold: HOLD,
        instruments: true,
        reestimate_every: 500,
    }
}

/// Rule 1: the loading ladder at h_K2, frictionless and at the pairing's highest `μ_f` (§16x).
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn step7_ladder() {
    let mut stage = Stage::new("rule 1, the loading ladder at h_K2");
    let wall = Wall::build(&stage.scene, stage.h_k2, None, [0.0; 3]);
    let model = wall.model(POISSON, 1.0, 1.0);
    let start = stage.wall_line("h_K2", &wall, &model);
    let budget = Stage::budget_loading(&wall, start);
    let [_, _, high] = corners();
    // Per rung: the peak push at the highest μ_f, the geometric share, and the patch at 0 and at the highest.
    let mut table: Vec<([f64; 4], bool)> = Vec::new();
    for rung in RUNGS {
        let (_, free_readings) = press(
            &mut stage,
            &wall,
            &model,
            start,
            spec(0.0, rung * budget),
            &format!("rung ×{rung}, μ_f 0"),
        );
        let (_, rough_readings) = press(
            &mut stage,
            &wall,
            &model,
            start,
            spec(high, rung * budget),
            &format!("rung ×{rung}, μ_f {high}"),
        );
        table.push((
            [
                rough_readings.peak,
                free_readings.share,
                free_readings.patch,
                rough_readings.patch,
            ],
            free_readings.patch.is_finite() && rough_readings.patch.is_finite(),
        ));
    }
    println!(
        "\nrule 1 [PUBLIC]: each reading's change from the rung before (peak push at μ_f {high}, geometric share, patch at 0, patch at {high}); valid"
    );
    for (k, (row, ok)) in table.iter().enumerate() {
        let changes = (k > 0).then(|| {
            (0..4)
                .map(|i| format!("{:+.2} %", 100.0 * change(table[k - 1].0[i], row[i])))
                .collect::<Vec<_>>()
                .join(", ")
        });
        println!(
            "  ×{}: {} ; valid {ok}",
            RUNGS[k],
            changes.unwrap_or_else(|| "the first rung".to_owned())
        );
    }
    let within =
        |a: usize, b: usize| (0..4).all(|i| change(table[a].0[i], table[b].0[i]).abs() <= K5_BAR);
    let chosen = (1..RUNGS.len() - 1).find(|&k| {
        within(k - 1, k) && within(k, k + 1) && table[k - 1..=k + 1].iter().all(|r| r.1)
    });
    match chosen {
        Some(k) => println!(
            "rule 1 ⇒ the loading time is ×{} the budget's [PUBLIC]; run the next stages with STEP7_LOADING={}",
            RUNGS[k], RUNGS[k]
        ),
        None => println!(
            "rule 1 ⇒ open: no rung up to ×8 meets it; D4 is read at ×16, a bound from below [PUBLIC]"
        ),
    }
}

/// §16z's size rule (§16x rule 2 with an eight-times wall), at `STEP7_LOADING`, over [`SIZES`] with a replicate at each
/// of [`REPLICATES`]; the interval rule's reading; and the loading check at the size used. Every wall is built and
/// checked before the first run.
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn step7_sizes() {
    let factor = env_number("STEP7_LOADING", 1.0);
    let mut stage = Stage::new(&format!("§16z's size rule, loading ×{factor}"));
    let corners = corners();
    let walls: Vec<Mounted> = SIZES
        .iter()
        .map(|&size| {
            let wall = Wall::build(&stage.scene, stage.h_k2 / size.cbrt(), None, [0.0; 3]);
            Mounted::new(&stage, wall, &format!("×{size}"), factor)
        })
        .collect();
    let shifted: Vec<Mounted> = REPLICATES
        .iter()
        .map(|&k| {
            let cell = walls[k].wall.cell;
            let target = stage.h_k2 / SIZES[k].cbrt();
            let wall = Wall::build(&stage.scene, target, Some(cell), [0.5 * cell; 3]);
            let label = format!("×{} replicate, lattice shifted half a cell", SIZES[k]);
            Mounted::new(&stage, wall, &label, factor)
        })
        .collect();
    let mut press_all = |mounted: &Mounted, name: &str| -> Vec<Cell> {
        corners
            .iter()
            .map(|&friction| {
                let spec = spec(friction, mounted.loading);
                let label = format!("{name}, μ_f {friction}");
                Cell::press(&mut stage, mounted.on(), spec, label).0
            })
            .collect()
    };
    let mut sizes: Vec<(Mounted, Vec<Cell>)> = walls
        .into_iter()
        .zip(SIZES)
        .map(|(mounted, size)| {
            let cells = press_all(&mounted, &format!("×{size}"));
            (mounted, cells)
        })
        .collect();
    let mut replicates: Vec<(Mounted, Vec<Cell>)> = shifted
        .into_iter()
        .zip(REPLICATES)
        .map(|(mounted, k)| {
            let cells = press_all(&mounted, &format!("×{} replicate", SIZES[k]));
            (mounted, cells)
        })
        .collect();
    // Each doubling's and each replicate's pair of readings per corner, at one interval.
    let doublings: Vec<Vec<(Readings, Readings)>> = (0..SIZES.len() - 1)
        .map(|k| {
            let (coarse, fine) = sizes.split_at_mut(k + 1);
            let ((a, a_cells), (b, b_cells)) = (&mut coarse[k], &mut fine[0]);
            (0..corners.len())
                .map(|c| {
                    at_one_interval(
                        &mut stage,
                        (&mut a_cells[c], a.on()),
                        (&mut b_cells[c], b.on()),
                    )
                })
                .collect()
        })
        .collect();
    let scatters: Vec<Vec<(Readings, Readings)>> = REPLICATES
        .iter()
        .zip(&mut replicates)
        .map(|(&k, (r, r_cells))| {
            let (a, a_cells) = &mut sizes[k];
            (0..corners.len())
                .map(|c| {
                    at_one_interval(
                        &mut stage,
                        (&mut a_cells[c], a.on()),
                        (&mut r_cells[c], r.on()),
                    )
                })
                .collect()
        })
        .collect();
    let deciding = |pairs: &[(Readings, Readings)], i: usize| {
        let (a, b) = pairs[DECIDING[i].1];
        change(deciding_reading(&a, i), deciding_reading(&b, i))
    };
    // A doubling is read against the scatter of the replicate at its coarser size, or the nearest coarser.
    let scatter_for = |k: usize| scatter_for(k, &REPLICATES);
    println!(
        "\n§16z's size rule [PUBLIC]: element counts over h_K2's {}",
        sizes[1..]
            .iter()
            .map(|s| format!(
                "{:.2}",
                s.0.model.element_count() as f64 / sizes[0].0.model.element_count() as f64
            ))
            .collect::<Vec<_>>()
            .join(", ")
    );
    let h: Vec<f64> = sizes.iter().map(|s| element_size(&s.0.model)).collect();
    let mut outcomes = vec![Outcome::Pass; doublings.len()];
    for (i, (name, c)) in DECIDING.iter().enumerate() {
        let mut moves = Vec::new();
        for (k, pairs) in doublings.iter().enumerate() {
            let (m, s) = (deciding(pairs, i), deciding(&scatters[scatter_for(k)], i));
            moves.push(format!(
                "×{} → ×{} {:+.2} % against the ×{} replicate's {:.2} % ({:?})",
                SIZES[k],
                SIZES[k + 1],
                100.0 * m,
                SIZES[REPLICATES[scatter_for(k)]],
                100.0 * s.abs(),
                outcome(&[(m, s)])
            ));
        }
        // The finest three sizes' own readings.
        let finest = SIZES.len() - 3;
        let read = |k: usize| deciding_reading(&sizes[k].1[*c].readings, i);
        let mixed =
            (finest..SIZES.len()).any(|k| sizes[k].1[*c].every() != sizes[finest].1[*c].every());
        let fit = Fit::through(
            [h[finest], h[finest + 1], h[finest + 2]],
            [read(finest), read(finest + 1), read(finest + 2)],
        );
        let fit_text = match fit {
            Err(why) => why.to_owned(),
            Ok(f) => format!(
                "order {:.2}; remaining error {}",
                f.order,
                (finest + 1..SIZES.len())
                    .map(|k| format!("×{} {:.2} %", SIZES[k], 100.0 * f.remaining(h[k])))
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
        };
        println!(
            "  {name}: {}; fit over the finest three: {fit_text}{}",
            moves.join("; "),
            if mixed {
                " (its sizes stood at different re-estimate intervals)"
            } else {
                ""
            }
        );
    }
    for (k, pairs) in doublings.iter().enumerate() {
        let scatter = &scatters[scatter_for(k)];
        let each: Vec<(f64, f64)> = (0..DECIDING.len())
            .map(|i| (deciding(pairs, i), deciding(scatter, i)))
            .collect();
        outcomes[k] = outcome(&each);
    }
    for (label, c) in [("the lowest", 1), ("the highest", 2)] {
        println!(
            "  beside, for Jon: the peak push at {label} μ_f: {}; the replicates {}",
            doublings
                .iter()
                .enumerate()
                .map(|(k, pairs)| format!(
                    "×{} → ×{} {:+.2} %",
                    SIZES[k],
                    SIZES[k + 1],
                    100.0 * change(pairs[c].0.peak, pairs[c].1.peak)
                ))
                .collect::<Vec<_>>()
                .join(", "),
            REPLICATES
                .iter()
                .zip(&scatters)
                .map(|(&k, pairs)| format!(
                    "at ×{} {:+.2} %",
                    SIZES[k],
                    100.0 * change(pairs[c].0.peak, pairs[c].1.peak)
                ))
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    let verdict = size_verdict(&outcomes);
    println!(
        "§16z's size rule: the doublings {} [PUBLIC]",
        outcomes
            .iter()
            .enumerate()
            .map(|(k, o)| format!("×{} → ×{} {o:?}", SIZES[k], SIZES[k + 1]))
            .collect::<Vec<_>>()
            .join(", ")
    );
    match verdict {
        SizeVerdict::Needs(k) => println!(
            "§16z's size rule ⇒ D1's readings need ×{} h_K2's element count{} [PUBLIC]",
            SIZES[k],
            if k > 0 && outcomes[k - 1] != Outcome::Fail {
                " (the doubling before has no verdict, so a coarser size may do)"
            } else {
                ""
            }
        ),
        SizeVerdict::NoVerdict(k, why) => println!(
            "§16z's size rule ⇒ no size: ×{}'s doubling is {why:?} [PUBLIC]",
            SIZES[k]
        ),
        SizeVerdict::Open => {
            let finest = *SIZES.last().unwrap();
            println!(
                "§16z's size rule ⇒ open: ×{finest} or finer (×{finest} is not judged: that needs a run at ×{}) \
                 [PUBLIC]",
                2.0 * finest
            );
        }
    }
    // The size the masked rule, the loading check and G6 use: the size picked, else the finest whose runs all stood.
    let used = match verdict {
        SizeVerdict::Needs(k) => Some(k),
        _ => (0..SIZES.len())
            .rev()
            .find(|&k| sizes[k].1.iter().all(Cell::stood)),
    };
    let Some(k) = used else {
        println!(
            "the size used: none, no size's runs all stood; the loading check is not run [PUBLIC]"
        );
        let cells: Vec<&Cell> = sizes.iter().chain(&replicates).flat_map(|s| &s.1).collect();
        interval_line(&cells);
        k4_line(&cells);
        return;
    };
    println!(
        "the size used by the masked rule, the loading check and G6: ×{}{} [PUBLIC]",
        SIZES[k],
        if matches!(verdict, SizeVerdict::Needs(_)) {
            ""
        } else {
            ", the finest whose runs all stood, not a size the rule picked"
        }
    );
    // The loading check (§16x rule 1's) at that size: its loading and twice it.
    let [_, _, high] = corners;
    let mut twice_cells: Vec<Cell> = Vec::new();
    let twice: Vec<(Readings, Readings)> = {
        let (mounted, cells) = &mut sizes[k];
        [0, 2]
            .into_iter()
            .map(|c| {
                let friction = corners[c];
                let spec = spec(friction, 2.0 * mounted.loading);
                let label = format!("×{}, twice the loading, μ_f {friction}", SIZES[k]);
                let (mut again, _) = Cell::press(&mut stage, mounted.on(), spec, label);
                let pair = at_one_interval(
                    &mut stage,
                    (&mut cells[c], mounted.on()),
                    (&mut again, mounted.on()),
                );
                twice_cells.push(again);
                pair
            })
            .collect()
    };
    let moves = [
        change(twice[1].0.peak, twice[1].1.peak),
        change(twice[0].0.share, twice[0].1.share),
        change(twice[0].0.patch, twice[0].1.patch),
        change(twice[1].0.patch, twice[1].1.patch),
    ];
    println!(
        "the loading check at ×{}: twice the loading moves the peak push at μ_f {high}, the geometric share, and the patch \
         at 0 and at {high} by {:+.2} %, {:+.2} %, {:+.2} %, {:+.2} % ⇒ {} [PUBLIC]",
        SIZES[k],
        100.0 * moves[0],
        100.0 * moves[1],
        100.0 * moves[2],
        100.0 * moves[3],
        loading_verdict(&moves)
    );
    let cells: Vec<&Cell> = sizes
        .iter()
        .chain(&replicates)
        .flat_map(|s| &s.1)
        .chain(&twice_cells)
        .collect();
    interval_line(&cells);
    k4_line(&cells);
}

/// The loading check's reading: open if a reading moves more than 5 %, else holding if every reading was read, else
/// not judged.
fn loading_verdict(moves: &[f64]) -> &'static str {
    if moves.iter().any(|m| m.abs() > K5_BAR) {
        "the loading is open at this size; the size rule and the masked rule hold at this loading only"
    } else if moves.iter().all(|m| m.is_finite()) {
        "the loading holds"
    } else {
        "not judged: a run did not stand"
    }
}

/// A deciding reading of §16x rule 2's: the patch at a corner, or the geometric share.
fn deciding_reading(readings: &Readings, i: usize) -> f64 {
    if i < 3 {
        readings.patch
    } else {
        readings.share
    }
}

/// A doubling's outcome over its deciding readings (§16z's size rule), each `(m, s)`: its change and the scatter of the
/// replicate it is read against. A reading fails if it moves past 5 % by more than `s`, passes if within 5 % by `s`,
/// and cannot tell between; one not read (a run that did not stand) is not judged. The doubling fails if a reading
/// fails, else is not judged if one is, else cannot tell if one cannot, else passes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Outcome {
    Pass,
    Fail,
    CannotTell,
    NotJudged,
}

fn outcome(readings: &[(f64, f64)]) -> Outcome {
    let each: Vec<Outcome> = readings
        .iter()
        .map(|&(m, s)| {
            if !(m.is_finite() && s.is_finite()) {
                Outcome::NotJudged
            } else if m.abs() - s.abs() > K5_BAR {
                Outcome::Fail
            } else if m.abs() + s.abs() <= K5_BAR {
                Outcome::Pass
            } else {
                Outcome::CannotTell
            }
        })
        .collect();
    [Outcome::Fail, Outcome::NotJudged, Outcome::CannotTell]
        .into_iter()
        .find(|o| each.contains(o))
        .unwrap_or(Outcome::Pass)
}

/// §16z's size rule over its doublings' outcomes, as indices into [`SIZES`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SizeVerdict {
    /// The coarsest size from which every doubling passes.
    Needs(usize),
    /// None does, and the finest doubling after the last that failed has no verdict: why.
    NoVerdict(usize, Outcome),
    /// None does, and the finest doubling fails.
    Open,
}

fn size_verdict(outcomes: &[Outcome]) -> SizeVerdict {
    if let Some(k) =
        (0..outcomes.len()).find(|&k| outcomes[k..].iter().all(|o| *o == Outcome::Pass))
    {
        return SizeVerdict::Needs(k);
    }
    let after = outcomes
        .iter()
        .rposition(|o| *o == Outcome::Fail)
        .map_or(0, |k| k + 1);
    outcomes[after..]
        .iter()
        .rposition(|o| *o != Outcome::Pass)
        .map_or(SizeVerdict::Open, |k| {
            SizeVerdict::NoVerdict(after + k, outcomes[after + k])
        })
}

/// `r(h) = r∞ + C hᵖ` through three sizes' readings (the stop rule's model, §15g step 2).
#[derive(Clone, Copy, Debug)]
struct Fit {
    order: f64,
    scale: f64,
    limit: f64,
}

impl Fit {
    /// The fit through `(h, r)`, or why there is none.
    fn through(h: [f64; 3], r: [f64; 3]) -> Result<Self, &'static str> {
        if !r.iter().all(|x| x.is_finite()) {
            return Err("a size's run did not stand, no fit");
        }
        let (d01, d12) = (r[0] - r[1], r[1] - r[2]);
        if d01 == 0.0 || d12 == 0.0 || d01.signum() != d12.signum() {
            return Err("not monotone, no fit");
        }
        let ratio =
            |p: f64| (h[0].powf(p) - h[1].powf(p)) / (h[1].powf(p) - h[2].powf(p)) - d01 / d12;
        let (mut low, mut high) = (0.05, 8.0);
        if ratio(low).signum() == ratio(high).signum() {
            return Err("no order in (0.05, 8)");
        }
        for _ in 0..100 {
            let mid = 0.5 * (low + high);
            if ratio(mid).signum() == ratio(low).signum() {
                low = mid;
            } else {
                high = mid;
            }
        }
        let order = 0.5 * (low + high);
        let scale = d12 / (h[1].powf(order) - h[2].powf(order));
        Ok(Self {
            order,
            scale,
            limit: r[2] - scale * h[2].powf(order),
        })
    }

    fn at(&self, h: f64) -> f64 {
        self.limit + self.scale * h.powf(self.order)
    }

    /// The error left at `h`, `|r(h) − r∞| / |r∞|`.
    fn remaining(&self, h: f64) -> f64 {
        ((self.at(h) - self.limit) / self.limit).abs()
    }
}

/// Rules 3, 5, 10 and 11 at h_K2 and `STEP7_LOADING`; the room on step 7's wall, the travel and the push's linearity
/// in friction (§16x).
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn step7_at_h_k2() {
    let factor = env_number("STEP7_LOADING", 1.0);
    let mut stage = Stage::new(&format!("rules 3, 5, 10 and 11 at h_K2, loading ×{factor}"));
    let wall = Wall::build(&stage.scene, stage.h_k2, None, [0.0; 3]);
    let model = wall.model(POISSON, 1.0, 1.0);
    let start = stage.wall_line("h_K2", &wall, &model);
    let loading = factor * Stage::budget_loading(&wall, start);
    let corners = corners();
    let [_, low, high] = corners;

    // The corners at ν 0.49, the frictionless one kept for rule 10 at the seat.
    let mut base = [Readings::default(); 3];
    for (k, friction) in corners.into_iter().enumerate() {
        let spec = spec(friction, loading);
        if friction > 0.0 {
            base[k] = press(
                &mut stage,
                &wall,
                &model,
                start,
                spec,
                &format!("ν 0.49, μ_f {friction}"),
            )
            .1;
            continue;
        }
        stage.set(&spec);
        let damping = stage.damping(&wall);
        let Stage {
            scene, obstacle, ..
        } = &mut stage;
        let press = Press {
            scene,
            model: &model,
            surface: surface_nodes(&model),
            start,
            loading,
        };
        // Gated as `press` gates the other corners, a hold that did not settle run again with a 0.4 s hold (rule 6);
        // a run whose readings do not stand feeds no rule, and rule 10 does not start from it. Rule 10's own holds are
        // not gated.
        let mut spec = spec;
        let (run, mut stepper) = loop {
            let (run, stepper) = run(
                &press,
                obstacle,
                |m, o| cpu::f32::CpuExecutor::new(m, o).unwrap(),
                damping,
                spec,
            );
            base[k] = run.readings(&press, obstacle);
            report("ν 0.49, μ_f 0", &run, &base[k], &press, obstacle);
            match falls_short(&run, &base[k]) {
                Some(reason) if reason.contains("rule 6") && spec.hold < 0.4 => {
                    println!("    ⚠ {reason}: run again with a 0.4 s hold [PUBLIC]");
                    spec.hold = 0.4;
                }
                _ => break (run, stepper),
            }
        };
        if let Some(reason) = falls_short(&run, &base[k]) {
            println!(
                "    ⚠ {reason}: its readings take no part in the rules, and rule 10 does not run [PUBLIC]"
            );
            base[k] = Readings::not_read();
            continue;
        }
        let n = obstacle.poses.len();
        let along = along_path(obstacle.poses[n - 2], obstacle.poses[n - 1], scene.tip).unwrap();
        let freedom = Freedom::new(scene, obstacle.poses[n - 1], along);
        free_scan(
            &press,
            obstacle,
            &mut stepper,
            &freedom,
            &|held, pose, obstacle| patch_at_pose(&model, held, pose, obstacle),
            "at the seat (the patch)",
        );
    }

    // Rule 10 at the pose of the frictionless run's largest 10 mm push, held until rule 6 settles.
    if base[0].share_centre.is_finite() {
        let spec = spec(0.0, loading);
        stage.set(&spec);
        let damping = stage.damping(&wall);
        let Stage {
            scene, obstacle, ..
        } = &mut stage;
        let press = Press {
            scene,
            model: &model,
            surface: surface_nodes(&model),
            start,
            loading,
        };
        let centre = press.time_at(base[0].share_centre);
        let (mut stepper, mut clock) = begin(
            &press,
            obstacle,
            |m, o| cpu::f32::CpuExecutor::new(m, o).unwrap(),
            damping,
            spec.reestimate_every,
        );
        let mut reads = Vec::new();
        drive(
            &press,
            obstacle,
            &mut stepper,
            centre,
            (&mut reads, &mut clock, false, &mut Collapse::default()),
        )
        .unwrap();
        let k = ((centre / obstacle.interval).floor() as usize).min(obstacle.poses.len() - 2);
        let (from, to) = (obstacle.poses[k], obstacle.poses[k + 1]);
        let advance = press.travel((k + 1) as f64 * obstacle.interval)
            - press.travel(k as f64 * obstacle.interval);
        let held_pose = obstacle.pose_at(centre);
        let along = along_path(from, to, scene.tip).unwrap();
        let freedom = Freedom::new(scene, held_pose, along);
        let push = |held: &Held, pose: Pose| {
            static_push(held.force, held.moment, pose, (from, to), advance).unwrap()
        };
        let zero = Vector4::zeros();
        let mut settled = None;
        let mut previous = f64::NAN;
        for window in 1..=4 {
            let held = probe(
                &press,
                obstacle,
                &mut stepper,
                &ramp(&freedom, &zero, &zero),
                (&mut reads, &mut clock),
            );
            let now = push(&held, freedom.pose(&zero));
            if change(previous, now).abs() <= SETTLE_BAR {
                settled = Some(window);
                break;
            }
            previous = now;
        }
        println!(
            "    rule 10 at the push's peak: held still, the static push settled within 1 % after {settled:?} holds [PUBLIC]"
        );
        free_scan(
            &press,
            obstacle,
            &mut stepper,
            &freedom,
            &|held, pose, _| push(held, pose),
            "at the push's peak (the static push)",
        );
    }

    // Rule 3: ν.
    let mut by_nu = vec![base];
    for poisson in &POISSONS[1..] {
        let model = wall.model(*poisson, 1.0, 1.0);
        by_nu.push(corners.map(|friction| {
            press(
                &mut stage,
                &wall,
                &model,
                start,
                spec(friction, loading),
                &format!("ν {poisson}, μ_f {friction}"),
            )
            .1
        }));
    }
    println!(
        "\nrule 3 [PUBLIC]: each reading's change per doubling of K (ν 0.49 → 0.495 → 0.4975)"
    );
    let readings_of = |r: &[Readings; 3]| {
        [
            r[0].patch, r[1].patch, r[2].patch, r[0].share, r[1].peak, r[2].peak,
        ]
    };
    let names = [
        "patch at 0",
        "patch at the lowest",
        "patch at the highest",
        "geometric share",
        "peak push at the lowest",
        "peak push at the highest",
    ];
    let (mut converging, mut judged) = (true, true);
    for (i, name) in names.iter().enumerate() {
        let [a, b, c] = [0, 1, 2].map(|n| readings_of(&by_nu[n])[i]);
        let (first, second) = (change(a, b), change(b, c));
        judged &= first.is_finite() && second.is_finite();
        converging &= second.abs() <= K5_BAR && second.abs() <= 0.5 * first.abs();
        println!(
            "  {name}: {:+.2} %, then {:+.2} %",
            100.0 * first,
            100.0 * second
        );
    }
    println!(
        "rule 3 ⇒ {} [PUBLIC]; which ν verdicts read at is Jon's call",
        if !judged {
            "not judged: a run behind a reading did not stand"
        } else if converging {
            "the readings converge in K here"
        } else {
            "ν's reading is open"
        }
    );

    // Rule 5: f32 against f64 on the patch at the highest μ_f.
    let wide = press(
        &mut stage,
        &wall,
        &model,
        start,
        Spec {
            wide: true,
            ..spec(high, loading)
        },
        &format!("f64, μ_f {high}"),
    )
    .1;
    let moved = change(base[2].patch, wide.patch);
    println!(
        "rule 5 [PUBLIC]: f32 against f64 on the patch at μ_f {high}: {:+.3} % ⇒ {}",
        100.0 * moved,
        if !moved.is_finite() {
            "not judged: a run behind it did not stand"
        } else if moved.abs() <= K3_BAR {
            "within K3's 0.5 %"
        } else {
            "⚠ past K3's 0.5 %: f32, which the GPU runs, is not trusted on it"
        }
    );

    // Rule 11: stiffness scaling.
    let stiff = wall.model(POISSON, 1.0, 2.0);
    let doubled = [0.0, high].map(|friction| {
        press(
            &mut stage,
            &wall,
            &stiff,
            start,
            spec(friction, loading),
            &format!("2μ, μ_f {friction}"),
        )
        .1
    });
    let scaled = [
        change(2.0 * base[0].share, doubled[0].share),
        change(2.0 * base[0].patch, doubled[0].patch),
        change(2.0 * base[2].peak, doubled[1].peak),
        change(2.0 * base[2].patch, doubled[1].patch),
    ];
    println!(
        "rule 11 [PUBLIC]: at 2μ over twice at μ: share {:+.2} %, patch at 0 {:+.2} %, peak push at {high} {:+.2} %, patch at {high} {:+.2} % ⇒ {}",
        100.0 * scaled[0],
        100.0 * scaled[1],
        100.0 * scaled[2],
        100.0 * scaled[3],
        if !scaled.iter().all(|s| s.is_finite()) {
            "not judged: a run behind it did not stand"
        } else if scaled.iter().all(|s| s.abs() <= SCALING_BAR) {
            "a verdict is 3 runs"
        } else {
            "a verdict is 5 runs"
        }
    );

    // The push's linearity in friction: the frictional push less the frictionless, at the highest over at the lowest.
    println!(
        "linearity [PUBLIC]: (peak(μ_high) − peak(0)) / (peak(μ_low) − peak(0)) {:.3} against μ_high / μ_low {:.3}",
        (base[2].peak - base[0].peak) / (base[1].peak - base[0].peak),
        high / low
    );
}

/// §16t's room through the fitted pose over its 64 poses, on step 7's wall's canal nodes and on the old wall's, both
/// through the scan's exact distance (§16x). The canal lies the inset inside the scan, so a canal node's room at a pose
/// is how far the posed scan puts it inside beyond that: its signed distance there plus the inset, negative where the
/// wall must make room.
fn room(scene: &Scene, model: &ExplicitModel, cell: f64) {
    let inset = scene.inset();
    let canal = |nodes: Vec<Point3<f64>>, within: f64| -> Vec<Point3<f64>> {
        nodes
            .into_iter()
            .filter(|&p| (scene.distance.signed(p) + inset).abs() < within)
            .collect()
    };
    let boundary = boundary_points(model);
    let step7 = canal(boundary.clone(), cell);
    let on_the_surface = step7
        .iter()
        .filter(|&&p| (scene.distance.signed(p) + inset).abs() < 1e-6)
        .count();
    let (old, _, _, old_boundary) = super::tests::sliding_product_scene().unwrap();
    let old_nodes = canal(
        old_boundary.iter().map(|p| Point3::from(*p)).collect(),
        old.cell_size_m,
    );
    let most = |nodes: &[Point3<f64>]| -> f64 {
        let length = scene.length();
        (1..=64)
            .map(|k| {
                let pose = scene
                    .path
                    .fitted(length * (1.0 - f64::from(k) / 64.0))
                    .unwrap();
                let inverse = pose.inverse();
                std::thread::scope(|s| {
                    nodes
                        .chunks(nodes.len().div_ceil(8).max(1))
                        .map(|chunk| {
                            s.spawn(move || {
                                chunk
                                    .iter()
                                    .map(|p| scene.distance.signed(inverse * p) + inset)
                                    .fold(f64::INFINITY, f64::min)
                            })
                        })
                        .collect::<Vec<_>>()
                        .into_iter()
                        .map(|h| h.join().unwrap())
                        .fold(f64::INFINITY, f64::min)
                })
            })
            .fold(f64::INFINITY, f64::min)
    };
    let (new_room, old_room) = (most(&step7), most(&old_nodes));
    println!(
        "room [LOCAL]: step 7's wall {:.4} mm over {} canal nodes of {} boundary nodes ({} within 1 µm of the inset), \
         the old wall {:.4} mm over {}; [PUBLIC] step 7's over the old's, both through the exact distance, {:.3}",
        1e3 * new_room,
        step7.len(),
        boundary.len(),
        on_the_surface,
        1e3 * old_room,
        old_nodes.len(),
        new_room / old_room
    );
}

/// The room on step 7's wall (§16v, §16x).
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn step7_room() {
    println!("\n══ step 7's first run on base_mold: the room on step 7's wall ══");
    let scene = Scene::load();
    let (h_k2, _) = h_k2();
    let wall = Wall::build(&scene, h_k2, None, [0.0; 3]);
    let model = wall.model(POISSON, 1.0, 1.0);
    room(&scene, &model, wall.cell);
}

/// The ×2 wall's frictionless run at `STEP7_LOADING`, which went non-finite near the seat (§16x's results): where its
/// first inverted elements lie, how far its step fell, and whether re-estimating the step ten times as often, or f64,
/// keeps it finite. One change at a time.
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn step7_blow_up() {
    let factor = env_number("STEP7_LOADING", 4.0);
    let mut stage = Stage::new(&format!(
        "the ×2 wall's frictionless run, loading ×{factor}"
    ));
    let wall = Wall::build(&stage.scene, stage.h_k2 / 2.0_f64.cbrt(), None, [0.0; 3]);
    let model = wall.model(POISSON, 1.0, 1.0);
    let start = stage.wall_line("×2", &wall, &model);
    let loading = factor * Stage::budget_loading(&wall, start);
    for (label, reestimate_every, wide) in [
        ("as run", 500, false),
        ("the step re-estimated every 50 steps", 50, false),
        ("f64", 500, true),
    ] {
        let spec = Spec {
            wide,
            ..spec(0.0, loading)
        };
        stage.set(&spec);
        let damping = stage.damping(&wall);
        let press = Press {
            scene: &stage.scene,
            model: &model,
            surface: surface_nodes(&model),
            start,
            loading,
        };
        if wide {
            diagnose(
                &press,
                &stage.obstacle,
                |m, o| cpu::f64::CpuExecutor::new(m, o).unwrap(),
                damping,
                reestimate_every,
                label,
            );
        } else {
            diagnose(
                &press,
                &stage.obstacle,
                |m, o| cpu::f32::CpuExecutor::new(m, o).unwrap(),
                damping,
                reestimate_every,
                label,
            );
        }
    }
}

/// One element's place and shape at rest, its `J` now, and the averaged `J` at its nodes, which sets the pressure
/// there (selective ANP, §15g step 1).
fn describe(press: &Press, e: usize, outputs: &PhaseOutputs) -> String {
    let dilation = outputs.dilations[e];
    let model = press.model;
    let positions = model.rest_positions();
    let mean_volume = model.rest_volumes().iter().sum::<f64>() / model.element_count() as f64;
    let nodes = model.elements()[e];
    let corners = nodes.map(|n| Point3::from(positions[n as usize]));
    let centre = Point3::from(
        (corners[0].coords + corners[1].coords + corners[2].coords + corners[3].coords) / 4.0,
    );
    let edges: Vec<f64> = (0..4)
        .flat_map(|a| ((a + 1)..4).map(move |b| (a, b)))
        .map(|(a, b)| (corners[a] - corners[b]).norm())
        .collect();
    let (short, long) = (
        edges.iter().copied().fold(f64::INFINITY, f64::min),
        edges.iter().copied().fold(0.0, f64::max),
    );
    let nodal = nodes.map(|n| {
        1.0 + sim_soft_explicit::f64::nodal_dilation(
            outputs.volume_changes[n as usize],
            model.node_rest_volumes()[n as usize],
        )
    });
    format!(
        "J {:.3}, its nodes' averaged J {:.3} to {:.3}; at {:.3} of the centreline from the seated tip, {:+.2} insets from the scan at rest, held nodes {}, on \
         the surface {}, rest volume over the mean {:.3}, shortest over longest edge {:.3}",
        1.0 + dilation,
        nodal.iter().copied().fold(f64::INFINITY, f64::min),
        nodal.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        press.scene.path.centreline().arc_of(centre) / press.scene.length(),
        press.scene.distance.signed(centre) / press.scene.inset(),
        nodes.iter().filter(|&&n| model.held()[n as usize]).count(),
        nodes
            .iter()
            .filter(|&&n| press.surface.binary_search(&n).is_ok())
            .count(),
        model.rest_volumes()[e] / mean_volume,
        short / long
    )
}

/// The CPU executors' power iteration with its vector (`top_mode_and_vector`), for [`diagnose`].
trait TopVector {
    fn top_vector(
        &self,
        iterations: usize,
        perturbation: f64,
        viscous_weight: f64,
    ) -> (TopMode, Vec<[f64; 3]>);
}

impl TopVector for cpu::f32::CpuExecutor {
    fn top_vector(
        &self,
        iterations: usize,
        perturbation: f64,
        viscous_weight: f64,
    ) -> (TopMode, Vec<[f64; 3]>) {
        self.top_mode_and_vector(iterations, perturbation, viscous_weight)
    }
}

impl TopVector for cpu::f64::CpuExecutor {
    fn top_vector(
        &self,
        iterations: usize,
        perturbation: f64,
        viscous_weight: f64,
    ) -> (TopMode, Vec<[f64; 3]>) {
        self.top_mode_and_vector(iterations, perturbation, viscous_weight)
    }
}

/// Each element's `J` over the mean of its nodes' averaged `J`: under 1 where the element has shrunk against the
/// volume its nodes keep.
fn against_nodes(model: &ExplicitModel, outputs: &PhaseOutputs) -> Vec<f64> {
    let nodal: Vec<f64> = outputs
        .volume_changes
        .iter()
        .zip(model.node_rest_volumes())
        .map(|(&change, &volume)| 1.0 + sim_soft_explicit::f64::nodal_dilation(change, volume))
        .collect();
    model
        .elements()
        .iter()
        .zip(&outputs.dilations)
        .map(|(nodes, &dilation)| {
            (1.0 + dilation) / (nodes.iter().map(|&n| nodal[n as usize]).sum::<f64>() / 4.0)
        })
        .collect()
}

/// Where the vector that sets the step sits, from its mass-weighted size at each node: the share on the node carrying
/// the most, the share on the most-compressed element's nodes, the most compressed of the top node's elements, and
/// the elements shrunk under a half and a quarter of their nodes' averaged `J` and where they lie.
fn locate(press: &Press, outputs: &PhaseOutputs, mode: TopMode, vector: &[[f64; 3]]) {
    let model = press.model;
    let weights: Vec<f64> = vector
        .iter()
        .zip(model.node_masses())
        .map(|(v, m)| m * (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]))
        .collect();
    let total: f64 = weights.iter().sum();
    let mut order: Vec<usize> = (0..weights.len()).collect();
    order.sort_by(|&a, &b| weights[b].total_cmp(&weights[a]));
    let top = order[0];
    let half = order
        .iter()
        .scan(0.0, |sum, &a| {
            *sum += weights[a];
            Some(*sum)
        })
        .position(|sum| sum >= 0.5 * total)
        .map_or(0, |i| i + 1);
    let most = (0..model.element_count())
        .min_by(|&a, &b| outputs.dilations[a].total_cmp(&outputs.dilations[b]))
        .unwrap();
    let at_top = (0..model.element_count())
        .filter(|&e| model.elements()[e].contains(&(top as u32)))
        .min_by(|&a, &b| outputs.dilations[a].total_cmp(&outputs.dilations[b]))
        .unwrap();
    let on_most = model.elements()[most]
        .iter()
        .map(|&n| weights[n as usize])
        .sum::<f64>()
        / total;
    println!(
        "      the vector that sets the step: its damping ratio {:.3}; the top node carries {:.3} of it, the most-compressed \
         element's nodes {:.3}; the most compressed of the top node's elements is the most compressed overall: {} [PUBLIC]; \
         {half} nodes carry half of it [LOCAL]",
        mode.damping_ratio(),
        weights[top] / total,
        on_most,
        at_top == most
    );
    println!(
        "      the most compressed of the top node's elements: {} [PUBLIC]",
        describe(press, at_top, outputs)
    );
    let shrunk = against_nodes(model, outputs);
    for share in [0.5, 0.25] {
        let arcs: Vec<f64> = (0..model.element_count())
            .filter(|&e| shrunk[e] < share)
            .map(|e| {
                let corners = model.elements()[e].map(|n| model.rest_positions()[n as usize]);
                let centre = Point3::from(
                    (0..4).fold(Vector3::zeros(), |sum, c| sum + Vector3::from(corners[c])) / 4.0,
                );
                press.scene.path.centreline().arc_of(centre) / press.scene.length()
            })
            .collect();
        println!(
            "      elements under {share} of their nodes' averaged J: {:.2e} of the wall's, from {:.3} to {:.3} of the \
             centreline from the seated tip [PUBLIC]; {} [LOCAL]",
            arcs.len() as f64 / model.element_count() as f64,
            arcs.iter().copied().fold(f64::INFINITY, f64::min),
            arcs.iter().copied().fold(f64::NEG_INFINITY, f64::max),
            arcs.len()
        );
    }
}

/// One frictionless run to the end of the loading, printing the smallest step over the rest step and the
/// most-compressed element where it fell, where the vector that sets the step sits then ([`locate`]), and at the first
/// read with an inverted element, the elements inverted then.
fn diagnose<E: Executor + TopVector>(
    press: &Press,
    obstacle: &Obstacle,
    make: impl FnOnce(&ExplicitModel, &Obstacle) -> E,
    damping: f64,
    reestimate_every: u64,
    label: &str,
) {
    let model = press.model;
    let rest = rest_step(model, obstacle);
    let top_speed = press.start / (0.9 * press.loading);
    let config = StepperConfig {
        monitor_every: ((READ_TRAVEL / (top_speed * rest)).floor() as u64).max(1),
        reestimate_every,
        ..StepperConfig::new(damping)
    };
    let mut stepper = Stepper::new(make(model, obstacle), config, 0.0);
    // The step the elastic top mode alone would give, without the viscosity: the vector at β = 0.
    let elastic_step = |executor: &E| {
        let (mode, _) = executor.top_vector(
            config.power_iterations,
            executor.epsilon().sqrt() * executor.shortest_edge(),
            0.0,
        );
        config.stable_step(mode.omega_squared, 0.0)
    };
    let elastic_rest = elastic_step(stepper.executor());
    // The loop's own first step, with the viscosity, on the same executor as `elastic_rest`.
    let first = stepper.dt();
    let (mut smallest, mut reported, mut seen) = (f64::INFINITY, false, 0);
    // Over every finite read: the least element J over its nodes' averaged J, and the loading's share then.
    let mut shrunk = (f64::INFINITY, f64::NAN);
    // The smallest step at a read, where it fell, the most-compressed element then, the state to locate the vector that
    // set it in, and the elastic step then over at rest.
    let mut lowest: Option<(f64, f64, String, PhaseOutputs, TopMode, Vec<[f64; 3]>, f64)> = None;
    let result = loop {
        if stepper.time() >= press.loading {
            break Ok(());
        }
        let step = stepper.step();
        smallest = smallest.min(stepper.dt() / rest);
        if stepper.samples().len() > seen {
            seen = stepper.samples().len();
            let sample = *stepper.samples().last().unwrap();
            let ratio = sample.dt / rest;
            if sample.monitors.finite() {
                let outputs = stepper.executor_mut().phase_outputs();
                let least = against_nodes(model, &outputs)
                    .into_iter()
                    .fold(f64::INFINITY, f64::min);
                if least < shrunk.0 {
                    shrunk = (least, sample.time / press.loading);
                }
            }
            if sample.monitors.finite() && lowest.as_ref().is_none_or(|l| ratio < 0.95 * l.0) {
                let outputs = stepper.executor_mut().phase_outputs();
                let (e, _) = outputs
                    .dilations
                    .iter()
                    .copied()
                    .enumerate()
                    .min_by(|a, b| a.1.total_cmp(&b.1))
                    .unwrap();
                let executor = stepper.executor();
                let (mode, vector) = executor.top_vector(
                    config.power_iterations,
                    executor.epsilon().sqrt() * executor.shortest_edge(),
                    2.0 / sample.dt,
                );
                let described = describe(press, e, &outputs);
                let elastic = elastic_step(executor) / elastic_rest;
                lowest = Some((
                    ratio,
                    sample.time / press.loading,
                    described,
                    outputs,
                    mode,
                    vector,
                    elastic,
                ));
            }
            if sample.monitors.inverted_element_steps > 0 && !reported {
                reported = true;
                let outputs = stepper.executor_mut().phase_outputs();
                let inverted: Vec<usize> = outputs
                    .dilations
                    .iter()
                    .enumerate()
                    .filter(|&(_, &d)| sim_soft_explicit::f64::is_inverted(d))
                    .map(|(e, _)| e)
                    .collect();
                println!(
                    "  {label}: the first inverted elements at {:.3} of the loading, the walk at {:.3} of its start, the \
                     step {:.3} of the rest step [PUBLIC]; {} inverted at that read [LOCAL]",
                    sample.time / press.loading,
                    1.0 - press.travel(sample.time) / press.start,
                    ratio,
                    inverted.len()
                );
                for &e in inverted.iter().take(6) {
                    println!("    element: {} [PUBLIC]", describe(press, e, &outputs));
                }
            }
        }
        if step.is_err() {
            break step;
        }
    };
    println!(
        "  {label}: {} at {:.3} of the loading; the smallest step over the rest step {:.3}; estimates {} [PUBLIC]",
        if result.is_ok() {
            "finite"
        } else {
            "⚠ not finite"
        },
        stepper.time() / press.loading,
        smallest,
        stepper.estimates()
    );
    println!(
        "    over every read, the least element J over its nodes' averaged J {:.3}, at {:.3} of the loading [PUBLIC]",
        shrunk.0, shrunk.1
    );
    if let Some((ratio, when, element, outputs, mode, vector, elastic)) = lowest {
        println!(
            "    at the smallest read step, {ratio:.3} of the rest step at {when:.3} of the loading, the most compressed: \
             {element} [PUBLIC]"
        );
        println!(
            "      the elastic top mode alone gives a step {elastic:.3} of its own at rest; at rest the viscosity takes the \
             step to {:.3} of the elastic one [PUBLIC]",
            first / elastic_rest
        );
        locate(press, &outputs, mode, &vector);
    }
}

/// Where the step falls near the seat (§16x's results): one change at a time from the ×2 wall mounted, each with the
/// step re-estimated every 50 steps so it runs through the loading: nothing held, h_K2's wall, and the ×4 wall.
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn step7_stiffening() {
    let factor = env_number("STEP7_LOADING", 4.0);
    let mut stage = Stage::new(&format!(
        "where the step falls near the seat, loading ×{factor}"
    ));
    for (label, size, held) in [
        ("×2, mounted", 2.0_f64, true),
        ("×2, nothing held", 2.0, false),
        ("h_K2, mounted", 1.0, true),
        ("×4, mounted", 4.0, true),
    ] {
        let wall = Wall::build(&stage.scene, stage.h_k2 / size.cbrt(), None, [0.0; 3]);
        let model = if held {
            wall.model(POISSON, 1.0, 1.0)
        } else {
            let lowering = Lowering {
                poisson: POISSON,
                viscous_time: ECOFLEX_00_30_VISCOUS_TIME,
            };
            lower(&wall.mesh, &wall.densities, lowering, &[])
                .unwrap()
                .model
        };
        let start = stage.wall_line(label, &wall, &model);
        let loading = factor * Stage::budget_loading(&wall, start);
        stage.set(&spec(0.0, loading));
        let damping = stage.damping(&wall);
        let press = Press {
            scene: &stage.scene,
            model: &model,
            surface: surface_nodes(&model),
            start,
            loading,
        };
        diagnose(
            &press,
            &stage.obstacle,
            |m, o| cpu::f32::CpuExecutor::new(m, o).unwrap(),
            damping,
            50,
            label,
        );
    }
}

/// The volumetric stabilization's stiffnesses over μ, the exploratory ladder (§16y), within the sources' span of 0.5 to
/// 25.
const LADDER: [f64; 5] = [0.0, 2.0, 4.0, 8.0, 25.0];

/// The volumetric stabilization (§16y), exploratory: at `STEP7_SIZE`'s wall, for each stiffness over μ in [`LADDER`],
/// the collapse ([`diagnose`], frictionless, the loop's re-estimate every 500 steps) and D1's readings at every
/// corner, each against the unstabilized wall's and the rung before.
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn step7_stabilized() {
    let factor = env_number("STEP7_LOADING", 1.0);
    let size = SIZES[env_number("STEP7_SIZE", 0.0) as usize];
    let mut stage = Stage::new(&format!(
        "the volumetric stabilization at ×{size} h_K2's elements, loading ×{factor}"
    ));
    let corners = corners();
    let wall = Wall::build(&stage.scene, stage.h_k2 / size.cbrt(), None, [0.0; 3]);
    let plain = wall.model(POISSON, 1.0, 1.0);
    let start = stage.wall_line(&format!("×{size}"), &wall, &plain);
    let loading = factor * Stage::budget_loading(&wall, start);
    let mut rows: Vec<(f64, [Readings; 3])> = Vec::new();
    for stiffness in LADDER {
        let model = plain
            .clone()
            .with_volumetric_stabilization(stiffness)
            .unwrap();
        let label = format!("×{size}, κ {stiffness} μ");
        stage.set(&spec(0.0, loading));
        let damping = stage.damping(&wall);
        let collapse = Press {
            scene: &stage.scene,
            model: &model,
            surface: surface_nodes(&model),
            start,
            loading,
        };
        diagnose(
            &collapse,
            &stage.obstacle,
            |m, o| cpu::f32::CpuExecutor::new(m, o).unwrap(),
            damping,
            500,
            &label,
        );
        let readings = corners.map(|friction| {
            press(
                &mut stage,
                &wall,
                &model,
                start,
                spec(friction, loading),
                &format!("{label}, μ_f {friction}"),
            )
            .1
        });
        rows.push((stiffness, readings));
    }
    let deciding = |r: &[Readings; 3]| {
        [
            r[0].patch, r[1].patch, r[2].patch, r[0].share, r[1].peak, r[2].peak,
        ]
    };
    let names = [
        "patch at 0",
        "patch at the lowest",
        "patch at the highest",
        "geometric share",
        "peak push at the lowest",
        "peak push at the highest",
    ];
    let first = deciding(&rows[0].1);
    println!(
        "
the stabilization's ladder at ×{size} [PUBLIC]:"
    );
    for pair in rows.windows(2) {
        let (before, after) = (deciding(&pair[0].1), deciding(&pair[1].1));
        let moves: Vec<String> = names
            .iter()
            .enumerate()
            .map(|(i, name)| {
                format!(
                    "{name} {:+.2} % ({:+.2} % from none)",
                    100.0 * change(before[i], after[i]),
                    100.0 * change(first[i], after[i])
                )
            })
            .collect();
        println!(
            "  κ {} μ → {} μ: {}",
            pair[0].0,
            pair[1].0,
            moves.join("; ")
        );
    }
}

/// §16z's masked rule (§16y rule 2 read again): at `STEP7_SIZE`'s wall (0 to 3 for one to eight times h_K2's elements),
/// at each corner, the element as it is; then the same with κ on only the elements that run drove under half their
/// nodes' averaged J, 2μ to start. An element under half in a masked run has its κ doubled, up to [`SOURCES_SPAN`] μ
/// or λ, or joins the mask at 2μ, over [`masked_rounds`] runs; the first run with none under half is the corner's
/// comparison, and if none clears, or those under half are at the cap already, the corner is not judged. Masked runs
/// start at the kept run's interval as it is, and the pair is read at one interval ([`at_one_interval`]). Then U20's
/// reading and U11's two sides.
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn step7_masked() {
    let factor = env_number("STEP7_LOADING", 4.0);
    let size = SIZES[env_number("STEP7_SIZE", 0.0) as usize];
    let mut stage = Stage::new(&format!(
        "§16z's masked rule at ×{size} h_K2's elements, loading ×{factor}"
    ));
    let wall = Wall::build(&stage.scene, stage.h_k2 / size.cbrt(), None, [0.0; 3]);
    let mounted = Mounted::new(&stage, wall, &format!("×{size}"), factor);
    let plain = &mounted.model;
    let materials = plain.materials();
    let rounds = masked_rounds(
        materials
            .iter()
            .map(|m| m.lambda / m.mu)
            .fold(f64::INFINITY, f64::min),
    );
    let corners = corners();
    // The runs as it is, and the masked runs apart: K4 is read on the element as it is.
    let (mut cells, mut masked_cells): (Vec<Cell>, Vec<Cell>) = (Vec::new(), Vec::new());
    let pairs = corners.map(|friction| {
        let (mut before, run) = Cell::press(
            &mut stage,
            mounted.on(),
            spec(friction, mounted.loading),
            format!("×{size}, as it is, μ_f {friction}"),
        );
        if !before.stood() {
            println!("    ×{size}, μ_f {friction}: the run as it is did not stand; nothing is masked [PUBLIC]");
            cells.push(before.clone());
            return (before.readings, Readings::not_read(), f64::NAN);
        }
        // Each masked element's κ over its μ: 2 to start; doubled, up to the cap, while it stays under half.
        let mut mask: BTreeMap<usize, f64> = run.collapse.elements.iter().map(|&e| (e, 2.0)).collect();
        if mask.is_empty() {
            println!("    ×{size}, μ_f {friction}: no element under half; nothing to mask [PUBLIC]");
            cells.push(before.clone());
            return (before.readings, before.readings, 0.0);
        }
        let mut accepted: Option<(Cell, ExplicitModel, f64)> = None;
        for round in 0..rounds {
            let stabilizations = (0..plain.element_count())
                .map(|e| mask.get(&e).map_or(0.0, |k| k * materials[e].mu))
                .collect();
            let model = plain
                .clone()
                .with_element_stabilizations(stabilizations)
                .unwrap();
            let on = Pressed {
                model: &model,
                ..mounted.on()
            };
            let label = format!("×{size}, κ on the collapsing elements, round {round}, μ_f {friction}");
            let (cell, run) = Cell::press(&mut stage, on, before.spec, label);
            masked_cells.push(cell.clone());
            if !cell.stood() {
                println!("    masked, round {round}: the run did not stand; the corner is not judged [PUBLIC]");
                break;
            }
            let kappa = mask.values().copied().fold(0.0, f64::max);
            println!(
                "    masked, round {round}: {:.1e} of the wall's elements, κ up to {kappa:.0} μ; the least element J over \
                 its nodes' {:.3}, under half after: {}; over as it is (every {} and {} steps): patch {:+.2} %, 10 mm \
                 push {:+.2} %, peak push {:+.2} % [PUBLIC]; {} masked, {} under half [LOCAL]",
                mask.len() as f64 / plain.element_count() as f64,
                run.collapse.least,
                if run.collapse.elements.is_empty() { "none" } else { "some" },
                before.every(),
                cell.every(),
                100.0 * change(before.readings.patch, cell.readings.patch),
                100.0 * change(before.readings.share, cell.readings.share),
                100.0 * change(before.readings.peak, cell.readings.peak),
                mask.len(),
                run.collapse.elements.len()
            );
            if run.collapse.elements.is_empty() {
                accepted = Some((cell, model, kappa));
                break;
            }
            let mut changed = false;
            for &e in &run.collapse.elements {
                let k = mask.entry(e).or_insert(0.0);
                let next = next_stiffening(*k, materials[e].lambda / materials[e].mu);
                changed |= next > *k;
                *k = next;
            }
            if !changed {
                println!("    the elements under half are at the cap already: the corner is not judged [PUBLIC]");
                break;
            }
            if round + 1 == rounds {
                println!("    still under half after {rounds} masked runs: the corner is not judged [PUBLIC]");
            }
        }
        let Some((mut after, model, kappa)) = accepted else {
            cells.push(before.clone());
            return (before.readings, Readings::not_read(), f64::NAN);
        };
        let on = Pressed {
            model: &model,
            ..mounted.on()
        };
        let (a, b) = at_one_interval(&mut stage, (&mut before, mounted.on()), (&mut after, on));
        cells.push(before);
        masked_cells.push(after);
        (a, b, kappa)
    });
    let deciding = |r: [&Readings; 3]| {
        [
            r[0].patch, r[1].patch, r[2].patch, r[0].share, r[1].peak, r[2].peak,
        ]
    };
    let (before, after) = (
        deciding([&pairs[0].0, &pairs[1].0, &pairs[2].0]),
        deciding([&pairs[0].1, &pairs[1].1, &pairs[2].1]),
    );
    let names = [
        "patch at 0",
        "patch at the lowest",
        "patch at the highest",
        "geometric share",
        "peak push at the lowest (for Jon)",
        "peak push at the highest (for Jon)",
    ];
    println!(
        "\n§16z's masked rule at ×{size} [PUBLIC]: masked over as it is, at one interval; the collapse cleared at κ {} μ \
         (μ_f 0, the lowest, the highest)",
        pairs
            .iter()
            .map(|p| if p.2.is_finite() {
                format!("{:.0}", p.2)
            } else {
                "none".to_owned()
            })
            .collect::<Vec<_>>()
            .join(", ")
    );
    for (i, name) in names.iter().enumerate() {
        println!("  {name}: {:+.2} %", 100.0 * change(before[i], after[i]));
    }
    let moves: Vec<f64> = (0..4).map(|i| change(before[i], after[i])).collect();
    if moves.iter().any(|m| !m.is_finite()) {
        println!(
            "§16z's masked rule at ×{size} ⇒ not judged at a corner: U20 goes back to Jon with each round's readings \
             [PUBLIC]"
        );
    } else if moves.iter().all(|m| m.abs() <= K5_BAR) {
        println!(
            "§16z's masked rule at ×{size} ⇒ every deciding reading's masked change is within 5 %: U20 stands [PUBLIC]"
        );
    } else {
        println!(
            "§16z's masked rule at ×{size} ⇒ a deciding reading's masked change reaches {:+.2} %: U20 goes back to Jon \
             [PUBLIC]",
            100.0
                * moves
                    .iter()
                    .copied()
                    .fold(0.0, |m: f64, x| if x.abs() > m.abs() { x } else { m })
        );
    }
    sides("the patch", pairs.map(|p| (p.0.patch, p.1.patch)));
    sides(
        "the push (the geometric share at μ_f 0, the peak push with friction; for Jon)",
        [
            (pairs[0].0.share, pairs[0].1.share),
            (pairs[1].0.peak, pairs[1].1.peak),
            (pairs[2].0.peak, pairs[2].1.peak),
        ],
    );
    interval_line(&cells.iter().chain(&masked_cells).collect::<Vec<_>>());
    k4_line(&cells.iter().collect::<Vec<_>>());
}

/// The masked rule's cap on κ over μ: the sources' span (§16y), where λ does not bind first.
const SOURCES_SPAN: f64 = 25.0;

/// A masked element's next κ over μ: 2 to join, else doubled, up to [`SOURCES_SPAN`] or `lambda` over μ.
fn next_stiffening(kappa: f64, lambda: f64) -> f64 {
    let cap = SOURCES_SPAN.min(lambda);
    if kappa == 0.0 {
        2.0_f64.min(cap)
    } else {
        (2.0 * kappa).min(cap)
    }
}

/// The masked runs: as many as take κ from 2μ to its cap by doubling, at the wall's least `λ/μ`.
fn masked_rounds(lambda: f64) -> usize {
    let cap = SOURCES_SPAN.min(lambda);
    1 + (cap / 2.0).log2().ceil().max(0.0) as usize
}

/// U11's two sides (fit plan: *fits* needs the corners' top under the limit, *too tight* their bottom over it): the
/// top and bottom over the corners, resisted over as it is, and which verdict each could turn (§16z).
fn sides(reading: &str, corners: [(f64, f64); 3]) {
    println!("{}", sides_text(reading, corners));
}

fn sides_text(reading: &str, corners: [(f64, f64); 3]) -> String {
    let Some((top, bottom)) = sides_of(corners) else {
        return format!(
            "U11 on {reading} [PUBLIC]: not judged, a corner's comparison did not stand"
        );
    };
    format!(
        "U11 on {reading} [PUBLIC]: the corners' top, resisted over as it is, {:+.2} %: {}; their bottom {:+.2} %: {}",
        100.0 * top,
        side(top, true),
        100.0 * bottom,
        side(bottom, false)
    )
}

/// The corners' top and bottom, each resisted over as it is, from `(as it is, resisted)` per corner; none if a corner
/// is not read.
fn sides_of(corners: [(f64, f64); 3]) -> Option<(f64, f64)> {
    if !corners.iter().all(|c| c.0.is_finite() && c.1.is_finite()) {
        return None;
    }
    let over = |pick: fn(f64, f64) -> f64| {
        let fold = |f: fn(&(f64, f64)) -> f64| corners.iter().map(f).reduce(pick).unwrap();
        change(fold(|c| c.0), fold(|c| c.1))
    };
    Some((over(f64::max), over(f64::min)))
}

/// What one side of U11's interval read `moved` (resisted over as it is) can do to the verdict that side decides: the
/// top a *fits*, the bottom a *too tight*. Read lower as it is, a *fits* can be false and a *too tight* missed; read
/// higher, the reverse.
fn side(moved: f64, top: bool) -> String {
    if moved == 0.0 {
        return "as it is and resisted read it alike".to_owned();
    }
    let lower = moved > 0.0;
    let verdict = if top { "*fits*" } else { "*too tight*" };
    format!(
        "as it is reads it {}, so a {verdict} within {:.2} % of a limit could be {}",
        if lower { "lower" } else { "higher" },
        100.0 * moved.abs(),
        if lower == top { "false" } else { "missed" }
    )
}

#[test]
fn a_doubling_is_read_against_its_replicates_scatter() {
    use Outcome::{CannotTell, Fail, NotJudged, Pass};
    assert_eq!(outcome(&[(0.02, 0.01)]), Pass);
    assert_eq!(outcome(&[(-0.02, -0.03)]), Pass);
    assert_eq!(outcome(&[(0.045, 0.01)]), CannotTell);
    assert_eq!(outcome(&[(-0.0555, 0.0301)]), CannotTell);
    assert_eq!(outcome(&[(0.07, 0.01)]), Fail);
    assert_eq!(outcome(&[(-0.07, 0.01)]), Fail);
    assert_eq!(outcome(&[(0.02, f64::NAN)]), NotJudged);
    assert_eq!(outcome(&[(0.02, 0.01), (0.045, 0.01)]), CannotTell);
    assert_eq!(outcome(&[(f64::NAN, 0.01), (0.045, 0.01)]), NotJudged);
    assert_eq!(outcome(&[(f64::NAN, 0.01), (0.07, 0.01)]), Fail);
}

#[test]
fn the_size_rule_picks_the_coarsest_size_from_which_every_doubling_passes() {
    use Outcome::{CannotTell as C, Fail as F, NotJudged as N, Pass as P};
    use SizeVerdict::{Needs, NoVerdict, Open};
    assert_eq!(size_verdict(&[F, F, P]), Needs(2));
    assert_eq!(size_verdict(&[P, F, P]), Needs(2));
    assert_eq!(size_verdict(&[F, P, P]), Needs(1));
    assert_eq!(size_verdict(&[P, P, P]), Needs(0));
    assert_eq!(size_verdict(&[C, P, P]), Needs(1));
    assert_eq!(size_verdict(&[F, F, F]), Open);
    assert_eq!(size_verdict(&[P, P, F]), Open);
    assert_eq!(size_verdict(&[F, N, F]), Open);
    assert_eq!(size_verdict(&[F, F, N]), NoVerdict(2, N));
    assert_eq!(size_verdict(&[F, F, C]), NoVerdict(2, C));
    assert_eq!(size_verdict(&[F, C, P]), Needs(2));
    assert_eq!(size_verdict(&[N, P, C]), NoVerdict(2, C));
}

#[test]
fn the_fit_recovers_a_power_law() {
    let h = [1.0, 0.8, 0.63];
    let law = |x: f64| 3.0 + 2.0 * x.powf(1.5);
    let fit = Fit::through(h, h.map(law)).unwrap();
    assert!((fit.order - 1.5).abs() < 1e-9 && (fit.limit - 3.0).abs() < 1e-9);
    assert!((fit.remaining(1.0) - 2.0 / 3.0).abs() < 1e-9);
    assert!((fit.remaining(0.5) - 2.0 * 0.5_f64.powf(1.5) / 3.0).abs() < 1e-9);
    assert!(Fit::through(h, [1.0, 2.0, 1.5]).is_err());
    assert!(Fit::through(h, [1.0, f64::NAN, 1.5]).is_err());
}

#[test]
fn u11s_sides_are_the_corners_top_and_bottom_resisted_over_as_it_is() {
    let (top, bottom) = sides_of([(1.0, 1.1), (2.0, 2.1), (3.0, 2.7)]).unwrap();
    assert!((top - (2.7 / 3.0 - 1.0)).abs() < 1e-12);
    assert!((bottom - 0.1).abs() < 1e-12);
    // The top and bottom are over the corners, whichever corner holds them.
    let (top, bottom) = sides_of([(3.0, 3.3), (1.0, 0.95), (2.0, 2.0)]).unwrap();
    assert!((top - 0.1).abs() < 1e-12 && (bottom - (-0.05)).abs() < 1e-12);
    assert!(sides_of([(1.0, f64::NAN), (2.0, 2.0), (3.0, 3.0)]).is_none());
}

#[test]
fn a_side_read_lower_as_it_is_can_make_a_fits_false_or_miss_a_too_tight() {
    assert!(
        side(0.02, true)
            .ends_with("reads it lower, so a *fits* within 2.00 % of a limit could be false")
    );
    assert!(
        side(-0.02, true)
            .ends_with("reads it higher, so a *fits* within 2.00 % of a limit could be missed")
    );
    assert!(side(0.02, false).ends_with("a *too tight* within 2.00 % of a limit could be missed"));
    assert!(side(-0.02, false).ends_with("a *too tight* within 2.00 % of a limit could be false"));
}

#[test]
fn the_masked_rule_doubles_kappa_from_2_mu_to_the_sources_span_or_lambda() {
    let mut kappa = 0.0;
    let mut schedule = Vec::new();
    for _ in 0..masked_rounds(49.0) {
        kappa = next_stiffening(kappa, 49.0);
        schedule.push(kappa);
    }
    assert_eq!(schedule, [2.0, 4.0, 8.0, 16.0, 25.0]);
    assert_eq!(next_stiffening(25.0, 49.0), 25.0);
    assert_eq!(masked_rounds(10.0), 4);
    assert_eq!(next_stiffening(8.0, 10.0), 10.0);
}

#[test]
fn a_doubling_is_read_against_the_replicate_at_or_nearest_below_its_coarser_size() {
    assert_eq!(scatter_for(0, &REPLICATES), 0);
    assert_eq!(scatter_for(1, &REPLICATES), 0);
    assert_eq!(scatter_for(2, &REPLICATES), 1);
    assert_eq!(scatter_for(3, &[0, 2]), 1);
}

#[test]
fn k4_fails_only_in_a_run_that_did_not_stop_and_held_its_gates() {
    assert!(k4_fails(false, true, true));
    assert!(!k4_fails(true, true, true));
    assert!(!k4_fails(false, false, true));
    assert!(!k4_fails(false, true, false));
}

/// A cell for the tests: its kept interval, whether it stood, and its corner.
fn test_cell(every: u64, stood: bool, friction: f64) -> Cell {
    Cell {
        spec: Spec {
            reestimate_every: every,
            ..spec(friction, 1.0)
        },
        label: format!("every {every}, μ_f {friction}"),
        k4: Vec::new(),
        k4_unread: Vec::new(),
        readings: if stood {
            Readings {
                patch: 100.0,
                share: 10.0,
                ..Readings::default()
            }
        } else {
            Readings::not_read()
        },
        fifty: None,
    }
}

#[test]
fn only_two_runs_that_stood_at_different_intervals_are_read_again() {
    let cell = |every: u64, stood: bool| test_cell(every, stood, 0.0);
    assert!(needs_one_interval(&cell(500, true), &cell(50, true)));
    assert!(needs_one_interval(&cell(50, true), &cell(500, true)));
    assert!(!needs_one_interval(&cell(500, true), &cell(500, true)));
    assert!(!needs_one_interval(&cell(50, true), &cell(50, true)));
    assert!(!needs_one_interval(&cell(500, false), &cell(50, true)));
    assert!(!needs_one_interval(&cell(500, true), &cell(50, false)));
    // The one run again is the one kept at the loop's interval.
    assert_eq!(run_again(&cell(500, true), &cell(50, true)), [true, false]);
    assert_eq!(run_again(&cell(50, true), &cell(500, true)), [false, true]);
    assert_eq!(
        run_again(&cell(500, true), &cell(500, true)),
        [false, false]
    );
    assert_eq!(
        run_again(&cell(500, false), &cell(50, true)),
        [false, false]
    );
}

#[test]
fn the_deciding_readings_are_the_patch_at_each_corner_and_the_share_at_0() {
    assert_eq!(DECIDING.map(|d| d.1), [0, 1, 2, 0]);
    let readings = Readings {
        patch: 1.0,
        share: 2.0,
        peak: 3.0,
        ..Readings::default()
    };
    assert_eq!(
        [0, 1, 2, 3].map(|i| deciding_reading(&readings, i)),
        [1.0, 1.0, 1.0, 2.0]
    );
}

#[test]
fn the_interval_rule_reads_the_patch_and_the_frictionless_share_against_k3() {
    let mut frictionless = test_cell(500, true, 0.0);
    frictionless.fifty = Some(Readings {
        patch: 100.0,
        share: 10.06,
        ..Readings::default()
    });
    let mut frictional = test_cell(500, true, 0.18);
    frictional.fifty = Some(Readings {
        patch: 100.2,
        share: 20.0,
        ..Readings::default()
    });
    let text = interval_text(&[&frictionless, &frictional, &test_cell(500, true, 0.0)]);
    assert!(
        text.contains("2 runs") && text.contains("0.600 %") && text.contains("past K3's 0.5 %"),
        "{text}"
    );
    // The frictional run's share is not a deciding reading.
    let text = interval_text(&[&frictional]);
    assert!(
        text.contains("0.200 %") && !text.contains("past K3"),
        "{text}"
    );
    // A run made again that did not stand is not counted, and none made again is not read.
    let mut unread = test_cell(500, true, 0.0);
    unread.fifty = Some(Readings::not_read());
    assert!(interval_text(&[&unread]).ends_with("not read"));
    assert!(interval_text(&[&test_cell(500, true, 0.0)]).ends_with("not read"));
}

#[test]
fn k4_names_the_runs_it_failed_in_and_those_it_was_not_read_in() {
    let clean = test_cell(50, true, 0.0);
    assert!(k4_text(&[&clean]).contains("holds"));
    let mut failed = test_cell(50, true, 0.104);
    failed.k4.push("a".to_owned());
    let mut stopped = test_cell(50, false, 0.18);
    stopped.k4_unread.push("b".to_owned());
    let text = k4_text(&[&clean, &failed, &stopped]);
    assert!(
        text.contains("FAILS") && text.contains(": a;") && text.ends_with(": b"),
        "{text}"
    );
    assert!(k4_text(&[&stopped]).contains("holds in every kept run that stood"));
}

#[test]
fn u11s_line_reads_the_top_as_a_fits_and_the_bottom_as_a_too_tight() {
    let text = sides_text("the patch", [(1.0, 1.1), (2.0, 2.1), (3.0, 3.03)]);
    let (top, bottom) = text.split_once("their bottom").unwrap();
    assert!(
        top.contains("*fits*") && !top.contains("*too tight*"),
        "{text}"
    );
    assert!(
        bottom.contains("*too tight*") && bottom.contains("+10.00 %"),
        "{text}"
    );
    assert!(
        sides_text("the patch", [(1.0, f64::NAN), (2.0, 2.0), (3.0, 3.0)])
            .ends_with("did not stand")
    );
}

#[test]
fn a_reading_past_5_percent_opens_the_loading_even_beside_one_not_read() {
    assert_eq!(
        loading_verdict(&[0.01, -0.02, 0.0, 0.049]),
        "the loading holds"
    );
    assert!(loading_verdict(&[0.01, -0.06, 0.0, 0.0]).starts_with("the loading is open"));
    assert!(loading_verdict(&[f64::NAN, 0.06, 0.0, 0.0]).starts_with("the loading is open"));
    assert!(loading_verdict(&[f64::NAN, 0.01, 0.0, 0.0]).starts_with("not judged"));
}

/// G6: a press timed at `STEP7_LOADING` and `STEP7_SIZE`, the probe's instruments off, under two step controls; and what
/// the cost rests on. With `STEP7_GPU` 1, on the GPU executor (§17b), under the product loop's step control alone.
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn step7_cost() {
    let factor = env_number("STEP7_LOADING", 1.0);
    let size = SIZES[env_number("STEP7_SIZE", 0.0) as usize];
    let gpu = env_number("STEP7_GPU", 0.0) > 0.5;
    let threads = std::env::var("RAYON_NUM_THREADS").unwrap_or_else(|_| "unset".to_owned());
    let executor = if gpu {
        "the GPU".to_owned()
    } else {
        format!("the CPU at {threads} threads")
    };
    let mut stage = Stage::new(&format!(
        "G6 at ×{size} h_K2's elements, loading ×{factor}, on {executor}"
    ));
    let wall = Wall::build(&stage.scene, stage.h_k2 / size.cbrt(), None, [0.0; 3]);
    let model = wall.model(POISSON, 1.0, 1.0);
    let start = stage.wall_line(&format!("×{size}"), &wall, &model);
    let loading = factor * Stage::budget_loading(&wall, start);
    // Two of the product loop's step controls (§15g's list): the loop's re-estimate every 500 steps with §16y rule 1's
    // re-run of a run that fails, and a fixed re-estimate every 50 steps.
    let mut cells: Vec<Cell> = Vec::new();
    let controls = [
        ("every 500 steps, with §16y rule 1's re-run", 500),
        ("every 50 steps", 50),
    ];
    for &(control, every) in &controls[..if gpu { 1 } else { 2 }] {
        let mut seconds = 0.0;
        let mut steps = 0;
        let mut standing = true;
        for friction in corners() {
            let spec = Spec {
                instruments: false,
                reestimate_every: every,
                gpu,
                ..spec(friction, loading)
            };
            let on = Pressed {
                wall: &wall,
                model: &model,
                start,
            };
            let label = format!("timed {control}, μ_f {friction}");
            let (cell, run) = Cell::press(&mut stage, on, spec, label);
            // §16y rule 1's and §16x rule 6's re-runs count with the attempts they replaced.
            let kept = run.clock.setup + run.clock.stepping;
            if run.earlier.1 > 0 {
                println!(
                    "    the attempts replaced: {:.1} s over {} steps [LOCAL]; their share of the corner's time {:.3}; \
                     the kept run, every {} steps with a {} s hold, over them in seconds per step {:.2} [PUBLIC]",
                    run.earlier.0,
                    run.earlier.1,
                    run.earlier.0 / (run.earlier.0 + kept),
                    run.spec.reestimate_every,
                    run.spec.hold,
                    (kept / run.steps as f64) / (run.earlier.0 / run.earlier.1 as f64)
                );
            }
            seconds += run.earlier.0 + kept;
            steps += run.earlier.1 + run.steps;
            standing &= cell.stood();
            cells.push(cell);
        }
        println!(
            "\nG6, {control} [LOCAL]: a press {seconds:.1} s over {steps} steps; [PUBLIC] over D4 {:.3} on \
             {executor}; a search of full verdicts at 1 and 2 insets over D4's 15 min {:.3} and {:.3}{}",
            seconds / D4_SECONDS,
            seconds / (3.0 * D4_SECONDS),
            2.0 * seconds / (3.0 * D4_SECONDS),
            if standing {
                ""
            } else {
                "; ⚠ a corner's kept run did not stand, so this is not a press's cost"
            }
        );
        // K1's per-step budget scaled by element count: the bar the GPU must meet, not a projection of it.
        let (budget, k1_elements) = k1_budget();
        let bar = budget * model.element_count() as f64 / k1_elements as f64;
        println!(
            "K1 [PUBLIC]: the same steps at K1's per-step budget, scaled by element count, over D4 {:.3}: the bar a GPU \
             must meet, not its projection",
            steps as f64 * bar / D4_SECONDS
        );
    }
    k4_line(&cells.iter().collect::<Vec<_>>());
    // What the cost rests on: the viscosity's range, and ν 0.495, as the rest step's factor on the steps.
    let rest = |m: &ExplicitModel| rest_step(m, &stage.obstacle);
    let at = rest(&model);
    for (label, other) in [
        ("η/μ × 0.74", wall.model(POISSON, 5.2 / 7.0, 1.0)),
        ("η/μ × 1.47", wall.model(POISSON, 10.3 / 7.0, 1.0)),
        ("ν 0.495", wall.model(0.495, 1.0, 1.0)),
        ("ν 0.4975", wall.model(0.4975, 1.0, 1.0)),
    ] {
        println!(
            "  the rest step's factor on the steps at {label}: {:.3} [PUBLIC]",
            at / rest(&other)
        );
    }
}
