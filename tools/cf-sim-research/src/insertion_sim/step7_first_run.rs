//! Step 7's first run on the product scan (soft-contact recon §16x): one press at the 5 mm inset, mounted at the
//! closed end, on the fitted path, with the rules §16x set before its runs. Five stages and two diagnostics, each an
//! ignored test:
//! - [`step7_ladder`], rule 1: the loading time, at h_K2;
//! - [`step7_sizes`], rule 2: the element size, at a loading, and rule 1 again at the size it picks;
//! - [`step7_at_h_k2`], rules 3, 5, 10 and 11 at h_K2 and a loading, with the push's linearity in friction;
//! - [`step7_room`], the room on step 7's wall;
//! - [`step7_cost`], G6: a press timed at a loading and a size, with the probe's own instruments off;
//! - [`step7_blow_up`] and [`step7_stiffening`], the element collapsing at the seated tip, one change at a time.
//!
//! Every run prints D1's readings, the sideways force and twist, G1 and G2 against the scan's exact distance, the band's
//! monitors, the validity gates and K4, and the seated window and contact work in quarters of the hold (rules 6–9). A
//! run whose readings do not stand ([`falls_short`]) feeds no rule, and a rule that would read it prints "not judged";
//! rule 10's probe holds are not gated.
//!
//! A stage's loading is `STEP7_LOADING`, in multiples of the budget's (default 1; §16x's runs used 4 for every stage but
//! the ladder); `step7_cost`'s size is `STEP7_SIZE`, 0, 1 or 2 for one, two and four times h_K2's element count
//! (default 0). Run each with
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

use std::f64::consts::TAU;
use std::time::Instant;

use cf_cap_planes::CapPlane;
use cf_device_types::SimDesign;
use mesh_types::IndexedMesh;
use nalgebra::{Isometry3, Matrix4, Point3, Translation3, UnitQuaternion, Vector3, Vector4};
use sim_soft::lowering::path::{Centreline, FittedPath, travelled};
use sim_soft::lowering::{
    Lowered, Lowering, PRODUCT_BAKE, Plane, SAMPLING_BAR, START_CLEARANCE, Skin, lower,
};
use sim_soft::obstacle::{SignedDistance, bake_surface};
use sim_soft::pairing::PAIRINGS;
use sim_soft::{Mesh, SdfMeshedTetMesh, VertexId, Yeoh};
use sim_soft_explicit::ExplicitModel;
use sim_soft_explicit::cpu;
use sim_soft_explicit::executor::{Executor, Monitors, Obstacle, PhaseOutputs, Snapshot};
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

/// Rule 2's element counts, over h_K2's.
const SIZES: [f64; 3] = [1.0, 2.0, 4.0];

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
    hold: f64,
    /// Whether the probe's instruments run: the exact G1 and G2 and the windows' snapshots.
    instruments: bool,
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
}

/// Step until `end`, reading every new monitor read's exact depth; then read once more if the last step was not read.
fn drive<E: Executor>(
    press: &Press,
    obstacle: &Obstacle,
    stepper: &mut Stepper<E>,
    end: f64,
    (reads, clock, instruments): (&mut Vec<Read>, &mut Clock, bool),
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
) -> (Stepper<E>, Clock) {
    let rest = rest_step(press.model, obstacle);
    let started = Instant::now();
    let executor = make(press.model, obstacle);
    let top_speed = press.start / (0.9 * press.loading);
    let monitor_every = ((READ_TRAVEL / (top_speed * rest)).floor() as u64).max(1);
    let config = StepperConfig {
        monitor_every,
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
    let (mut stepper, mut clock) = begin(press, obstacle, make, damping);
    let mut reads = Vec::new();
    let mut quarters = Vec::new();
    let mut stopped = drive(
        press,
        obstacle,
        &mut stepper,
        spec.loading,
        (&mut reads, &mut clock, spec.instruments),
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
            (&mut reads, &mut clock, spec.instruments),
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
        if spec.wide { "f64" } else { "f32" },
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
    let (run, readings) = if spec.wide {
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
        Some(reason) if reason.contains("rule 6") && spec.hold < 0.4 => {
            println!("    ⚠ {reason}: run again with a 0.4 s hold [PUBLIC]");
            self::press(
                stage,
                wall,
                model,
                start,
                Spec { hold: 0.4, ..spec },
                &format!("{label}, 0.4 s hold"),
            )
        }
        Some(reason) => {
            println!("    ⚠ {reason}: its readings take no part in the rules [PUBLIC]");
            (run, Readings::not_read())
        }
    }
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
        (reads, clock, false),
    )
    .unwrap();
    let first = stepper.samples().len();
    stepper.open_window();
    let end = start + MOVE_TIME + PROBE_HOLD + PROBE_READ;
    drive(press, obstacle, stepper, end, (reads, clock, false)).unwrap();
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
        hold: HOLD,
        instruments: true,
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

/// Rule 2: the element size, at `STEP7_LOADING`; and rule 1's check at the size it picks (§16x).
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn step7_sizes() {
    let factor = env_number("STEP7_LOADING", 1.0);
    let mut stage = Stage::new(&format!("rule 2, the element size, loading ×{factor}"));
    let corners = corners();
    let deciding = |r: &[Readings; 3]| [r[0].patch, r[1].patch, r[2].patch, r[0].share];
    let mut sizes: Vec<(f64, usize, [Readings; 3], Wall)> = Vec::new();
    for size in SIZES {
        let wall = Wall::build(&stage.scene, stage.h_k2 / size.cbrt(), None, [0.0; 3]);
        let model = wall.model(POISSON, 1.0, 1.0);
        let start = stage.wall_line(&format!("×{size}"), &wall, &model);
        let loading = factor * Stage::budget_loading(&wall, start);
        let readings = corners.map(|friction| {
            press(
                &mut stage,
                &wall,
                &model,
                start,
                spec(friction, loading),
                &format!("×{size}, μ_f {friction}"),
            )
            .1
        });
        sizes.push((element_size(&model), model.element_count(), readings, wall));
    }
    let replicate = {
        let cell = sizes[0].3.cell;
        let wall = Wall::build(&stage.scene, stage.h_k2, Some(cell), [0.5 * cell; 3]);
        let model = wall.model(POISSON, 1.0, 1.0);
        let start = stage.wall_line("h_K2 replicate, lattice shifted half a cell", &wall, &model);
        let loading = factor * Stage::budget_loading(&wall, start);
        corners.map(|friction| {
            press(
                &mut stage,
                &wall,
                &model,
                start,
                spec(friction, loading),
                &format!("replicate, μ_f {friction}"),
            )
            .1
        })
    };
    let names = [
        "patch at 0",
        "patch at the lowest",
        "patch at the highest",
        "geometric share",
    ];
    let values: Vec<[f64; 4]> = sizes.iter().map(|s| deciding(&s.2)).collect();
    let scatter = deciding(&replicate);
    println!(
        "\nrule 2 [PUBLIC]: element counts over h_K2's {:.2} and {:.2}",
        sizes[1].1 as f64 / sizes[0].1 as f64,
        sizes[2].1 as f64 / sizes[0].1 as f64
    );
    for (i, name) in names.iter().enumerate() {
        println!(
            "  {name}: ×1 → ×2 {:+.2} %, ×2 → ×4 {:+.2} %, ×1 → ×4 {:+.2} %; the replicate against ×1 {:+.2} %; fit: {}",
            100.0 * change(values[0][i], values[1][i]),
            100.0 * change(values[1][i], values[2][i]),
            100.0 * change(values[0][i], values[2][i]),
            100.0 * change(values[0][i], scatter[i]),
            fit(
                &sizes.iter().map(|s| s.0).collect::<Vec<_>>(),
                &values.iter().map(|v| v[i]).collect::<Vec<_>>()
            )
        );
    }
    for (label, k) in [("the lowest", 1), ("the highest", 2)] {
        println!(
            "  beside, for Jon: the peak push at {label} μ_f: ×1 → ×2 {:+.2} %, ×2 → ×4 {:+.2} %, ×1 → ×4 {:+.2} %; replicate {:+.2} %",
            100.0 * change(sizes[0].2[k].peak, sizes[1].2[k].peak),
            100.0 * change(sizes[1].2[k].peak, sizes[2].2[k].peak),
            100.0 * change(sizes[0].2[k].peak, sizes[2].2[k].peak),
            100.0 * change(sizes[0].2[k].peak, replicate[k].peak)
        );
    }
    let noisy = (0..4).any(|i| change(values[0][i], scatter[i]).abs() >= K5_BAR);
    // A size passes if every deciding reading moves at most 5 % over its doubling, fails if any moves more, and is not
    // judged otherwise: a run behind a reading did not stand.
    let doubling = |k: usize| {
        let moves: Vec<f64> = (0..4)
            .map(|i| change(values[k][i], values[k + 1][i]))
            .collect();
        if moves.iter().any(|m| m.abs() > K5_BAR) {
            Some(false)
        } else if moves.iter().all(|m| m.is_finite()) {
            Some(true)
        } else {
            None
        }
    };
    let outcomes = [doubling(0), doubling(1)];
    let chosen = (0..2).find(|&k| outcomes[k] == Some(true));
    if noisy {
        println!("rule 2 ⇒ the replicate's scatter is 5 % or more: the rule cannot tell [PUBLIC]");
    }
    match (chosen, outcomes.iter().position(Option::is_none)) {
        (Some(k), _) => println!(
            "rule 2 ⇒ D1's readings need ×{} h_K2's element count [PUBLIC]",
            SIZES[k]
        ),
        (None, Some(k)) => println!(
            "rule 2 ⇒ not judged at ×{}: a run behind a reading did not stand [PUBLIC]",
            SIZES[k]
        ),
        (None, None) => println!(
            "rule 2 ⇒ open: ×4 or finer (×4 is not judged: that needs a run at ×8); D4 at the fit's size, labelled so \
             [PUBLIC]"
        ),
    }
    // Rule 1 again at the size rule 2 picks: its loading and twice it.
    if let Some(k) = chosen.filter(|_| !noisy) {
        let wall = &sizes[k].3;
        let model = wall.model(POISSON, 1.0, 1.0);
        let start = stage.wall_line("rule 1's check", wall, &model);
        let loading = factor * Stage::budget_loading(wall, start);
        let [_, _, high] = corners;
        let free = press(
            &mut stage,
            wall,
            &model,
            start,
            spec(0.0, 2.0 * loading),
            "twice the loading, μ_f 0",
        )
        .1;
        let rough = press(
            &mut stage,
            wall,
            &model,
            start,
            spec(high, 2.0 * loading),
            &format!("twice the loading, μ_f {high}"),
        )
        .1;
        let at = &sizes[k].2;
        let moves = [
            change(at[2].peak, rough.peak),
            change(at[0].share, free.share),
            change(at[0].patch, free.patch),
            change(at[2].patch, rough.patch),
        ];
        println!(
            "rule 1 at ×{}: twice the loading moves the readings {:+.2} %, {:+.2} %, {:+.2} %, {:+.2} % ⇒ {} [PUBLIC]",
            SIZES[k],
            100.0 * moves[0],
            100.0 * moves[1],
            100.0 * moves[2],
            100.0 * moves[3],
            if moves.iter().all(|m| m.abs() <= K5_BAR) {
                "the loading holds"
            } else {
                "rule 1 is read again at this size"
            }
        );
    }
}

/// `r(h) = r∞ + C hᵖ` through three sizes' readings: the order, and each size's remaining error `|r − r∞| / |r∞|`.
fn fit(h: &[f64], r: &[f64]) -> String {
    if !r.iter().all(|x| x.is_finite()) {
        return "a size's run is not valid, no fit".to_owned();
    }
    let (d01, d12) = (r[0] - r[1], r[1] - r[2]);
    if d01 == 0.0 || d12 == 0.0 || d01.signum() != d12.signum() {
        return "not monotone, no fit".to_owned();
    }
    let ratio = |p: f64| (h[0].powf(p) - h[1].powf(p)) / (h[1].powf(p) - h[2].powf(p)) - d01 / d12;
    let (mut low, mut high) = (0.05, 8.0);
    if ratio(low).signum() == ratio(high).signum() {
        return "no order in (0.05, 8)".to_owned();
    }
    for _ in 0..100 {
        let mid = 0.5 * (low + high);
        if ratio(mid).signum() == ratio(low).signum() {
            low = mid;
        } else {
            high = mid;
        }
    }
    let p = 0.5 * (low + high);
    let c = d12 / (h[1].powf(p) - h[2].powf(p));
    let limit = r[2] - c * h[2].powf(p);
    format!(
        "order {p:.2}, remaining error ×1 {:.2} %, ×2 {:.2} %, ×4 {:.2} %",
        100.0 * ((r[0] - limit) / limit).abs(),
        100.0 * ((r[1] - limit) / limit).abs(),
        100.0 * ((r[2] - limit) / limit).abs()
    )
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
        );
        let mut reads = Vec::new();
        drive(
            &press,
            obstacle,
            &mut stepper,
            centre,
            (&mut reads, &mut clock, false),
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

/// One frictionless run to the end of the loading, printing the smallest step over the rest step and the
/// most-compressed element where it fell, and at the first read with an inverted element, the elements inverted then.
fn diagnose<E: Executor>(
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
    let (mut smallest, mut reported, mut seen) = (f64::INFINITY, false, 0);
    // The smallest step at a read, where it fell, and the most-compressed element then.
    let mut lowest: Option<(f64, f64, String)> = None;
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
            if sample.monitors.finite() && lowest.as_ref().is_none_or(|l| ratio < 0.95 * l.0) {
                let outputs = stepper.executor_mut().phase_outputs();
                let (e, _) = outputs
                    .dilations
                    .iter()
                    .copied()
                    .enumerate()
                    .min_by(|a, b| a.1.total_cmp(&b.1))
                    .unwrap();
                lowest = Some((
                    ratio,
                    sample.time / press.loading,
                    describe(press, e, &outputs),
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
    if let Some((ratio, when, element)) = lowest {
        println!(
            "    at the smallest read step, {ratio:.3} of the rest step at {when:.3} of the loading, the most compressed: \
             {element} [PUBLIC]"
        );
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

/// G6: a press timed at `STEP7_LOADING` and `STEP7_SIZE`, the probe's instruments off; and what the cost rests on.
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn step7_cost() {
    let factor = env_number("STEP7_LOADING", 1.0);
    let size = SIZES[env_number("STEP7_SIZE", 0.0) as usize];
    let threads = std::env::var("RAYON_NUM_THREADS").unwrap_or_else(|_| "unset".to_owned());
    let mut stage = Stage::new(&format!(
        "G6 at ×{size} h_K2's elements, loading ×{factor}, {threads} threads"
    ));
    let wall = Wall::build(&stage.scene, stage.h_k2 / size.cbrt(), None, [0.0; 3]);
    let model = wall.model(POISSON, 1.0, 1.0);
    let start = stage.wall_line(&format!("×{size}"), &wall, &model);
    let loading = factor * Stage::budget_loading(&wall, start);
    let mut seconds = 0.0;
    let mut steps = 0;
    for friction in corners() {
        let spec = Spec {
            instruments: false,
            ..spec(friction, loading)
        };
        let (run, _) = press(
            &mut stage,
            &wall,
            &model,
            start,
            spec,
            &format!("timed, μ_f {friction}"),
        );
        seconds += run.clock.setup + run.clock.stepping;
        steps += run.steps;
    }
    println!(
        "\nG6 [LOCAL]: a press {seconds:.1} s over {steps} steps; [PUBLIC] over D4 {:.3} on the CPU at {threads} threads; \
         a search of full verdicts at 1 and 2 insets over D4's 15 min {:.3} and {:.3}",
        seconds / D4_SECONDS,
        seconds / (3.0 * D4_SECONDS),
        2.0 * seconds / (3.0 * D4_SECONDS)
    );
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
            "  steps at {label} over these: {:.3} [PUBLIC]",
            at / rest(&other)
        );
    }
    // K1's per-step budget scaled by element count: the bar the GPU must meet, not a projection of it.
    let (budget, k1_elements) = k1_budget();
    let bar = budget * model.element_count() as f64 / k1_elements as f64;
    println!(
        "K1 [PUBLIC]: the same steps at K1's per-step budget, scaled by element count, over D4 {:.3}: the bar a GPU must \
         meet, not its projection",
        steps as f64 * bar / D4_SECONDS
    );
}
