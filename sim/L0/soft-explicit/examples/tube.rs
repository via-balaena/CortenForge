//! One run of the tube on the mandrel (plan §15b, §16i), printed as one line:
//! G2's penetration against the grid and the true surface, the band's gap,
//! K2's errors and its pair-averaged ring levels, K5's readings, the validity
//! gates, the loaded step factor, and the run's cost.
//!
//! `cargo run --release -p sim-soft-explicit --example tube --
//! <10k|50k|100k> <case> <friction> <f32|f64> <grid> <hold> <loading> <stiffness> <viscous>`
//! (defaults: 10k 0 0 f32 20 0.2 10 1, and Ecoflex 00-30's `η/μ`). `case` indexes
//! `fixtures::golden::THICK_TUBE`; the grid's cell is A/`grid`; `hold` is
//! the hold after loading, in seconds (plan §15b: 0.2); `loading` is the
//! loading time in shear periods `T_s` of the unscaled material (plan §15c's
//! ladder starts at 10); `stiffness` multiplies μ, and so λ and the viscosity
//! (plan 15d.10); `viscous` is the Kelvin–Voigt `η/μ` in seconds (plan §16p).
//! With friction on the free tube, a frictionless companion run gives the
//! Coulomb push ratio (plan 15d.7). Set `RAYON_NUM_THREADS` so the times are comparable.

#![allow(missing_docs, clippy::unwrap_used, clippy::cast_precision_loss)]

use std::time::Instant;

use sim_soft_explicit::ExplicitModel;
use sim_soft_explicit::cpu;
use sim_soft_explicit::executor::{Executor, Obstacle};
use sim_soft_explicit::f64 as shared;
use sim_soft_explicit::fixtures::golden::THICK_TUBE;
use sim_soft_explicit::fixtures::tube::{
    ECOFLEX_00_30_VISCOUS_TIME, Insertion, Mesh, Tube, TubeResult, TubeRun, Walls, node_pressures,
    tributary_area,
};
use sim_soft_explicit::stepping::StepperConfig;

const MU: f64 = 23.0e3;
const DENSITY: f64 = 1070.0;

/// The command line: which run, at which precision.
struct Request {
    run: TubeRun,
    case_index: usize,
    wide: bool,
}

fn request() -> Request {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let arg = |i: usize, default: &str| args.get(i).cloned().unwrap_or_else(|| default.to_owned());
    let mesh = match arg(0, "10k").as_str() {
        "10k" => Mesh::TenK,
        "50k" => Mesh::FiftyK,
        "100k" => Mesh::HundredK,
        other => {
            eprintln!("unknown mesh {other}: use 10k, 50k or 100k");
            std::process::exit(2);
        }
    };
    let case_index: usize = arg(1, "0").parse().unwrap();
    let divisions: f64 = arg(4, "20").parse().unwrap();
    let periods: f64 = arg(6, "10").parse().unwrap();
    let stiffness: f64 = arg(7, "1").parse().unwrap();
    let mut insertion = Insertion::plan(periods * TubeRun::shear_period(MU, DENSITY));
    insertion.hold = arg(5, "0.2").parse().unwrap();
    Request {
        run: TubeRun {
            mesh,
            case: THICK_TUBE[case_index],
            mu: stiffness * MU,
            viscous_time: args
                .get(8)
                .map_or(ECOFLEX_00_30_VISCOUS_TIME, |v| v.parse().unwrap()),
            density: DENSITY,
            insertion,
            window: 0.1,
            friction: arg(2, "0").parse().unwrap(),
            grid_cell: Tube::plan(mesh).inner_radius / divisions,
        },
        case_index,
        wide: arg(3, "f32") == "f64",
    }
}

/// One cold estimate of the stable step, timed alone: each of the loop's
/// estimates costs about this.
fn estimate_seconds(model: &ExplicitModel, obstacle: &Obstacle, wide: bool) -> f64 {
    let iterations = StepperConfig::new(0.0).power_iterations;
    let mut executor: Box<dyn Executor> = if wide {
        Box::new(cpu::f64::CpuExecutor::new(model, obstacle).unwrap())
    } else {
        Box::new(cpu::f32::CpuExecutor::new(model, obstacle).unwrap())
    };
    let perturbation = executor.epsilon().sqrt() * executor.shortest_edge();
    let started = Instant::now();
    // Any positive weight costs the viscous evaluations a run's estimates make.
    executor.estimate_top_mode(iterations, perturbation, 1.0);
    started.elapsed().as_secs_f64()
}

/// The element size the stop rule uses: (mean tet volume)^⅓.
fn element_size(model: &ExplicitModel) -> f64 {
    let rest = model.rest_positions();
    let volume: f64 = model
        .elements()
        .iter()
        .map(|element| {
            let [origin, first, second, third] = element.map(|n| rest[n as usize]);
            let edge = |p: [f64; 3]| [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]];
            shared::vec3_dot(edge(first), shared::vec3_cross(edge(second), edge(third))).abs() / 6.0
        })
        .sum();
    (volume / model.elements().len() as f64).cbrt()
}

/// G2 at the end state, over every surface node in the mandrel's body frame.
struct EndPenetration {
    /// The deepest node against the true surface.
    truth: f64,
    /// The deepest node against the baked grid.
    grid: f64,
    /// The range of (true − grid) depth over the nodes inside the true surface.
    bias: (f64, f64),
    /// The deepest node's body-frame z (the tip is at 0).
    deepest_z: f64,
}

fn end_penetration(
    run: &TubeRun,
    tube: &Tube,
    model: &ExplicitModel,
    obstacle: &Obstacle,
    result: &TubeResult,
) -> EndPenetration {
    let mandrel = run.mandrel(tube);
    let tip = run.insertion.tip(run.insertion.end());
    let rest = model.rest_positions();
    let mut surface: Vec<u32> = model
        .surface_triangles()
        .iter()
        .flatten()
        .copied()
        .collect();
    surface.sort_unstable();
    surface.dedup();
    let mut end = EndPenetration {
        truth: 0.0,
        grid: 0.0,
        bias: (f64::INFINITY, f64::NEG_INFINITY),
        deepest_z: f64::NAN,
    };
    for &node in &surface {
        let world = shared::vec3_add(
            rest[node as usize],
            result.snapshot.displacements[node as usize],
        );
        let body = [world[0], world[1], world[2] - tip];
        let (truth, grid) = (-mandrel.distance(body), -obstacle.sample(body).distance);
        if truth > end.truth {
            end.truth = truth;
            end.deepest_z = body[2];
        }
        end.grid = end.grid.max(grid);
        if truth > 0.0 {
            end.bias = (end.bias.0.min(truth - grid), end.bias.1.max(truth - grid));
        }
    }
    end
}

/// K5's seated reading (fit plan D1): the area-weighted 95th percentile of
/// the window-mean contact pressure, the pressure the most-squeezed 5 % of
/// the contact area is at or above.
fn seated_percentile(model: &ExplicitModel, result: &TubeResult) -> f64 {
    let snapshot = &result.snapshot;
    let contact: Vec<u32> = (0_u32..)
        .zip(&snapshot.normal_force_sums)
        .filter(|&(_, &sum)| sum > 0.0)
        .map(|(node, _)| node)
        .collect();
    let pressures = node_pressures(model, snapshot, &contact);
    let mut weighted: Vec<(f64, f64)> = contact
        .iter()
        .zip(pressures)
        .map(|(&node, pressure)| (pressure, tributary_area(model, snapshot, node as usize)))
        .collect();
    weighted.sort_by(|a, b| b.0.total_cmp(&a.0));
    let total: f64 = weighted.iter().map(|&(_, area)| area).sum();
    let mut covered = 0.0;
    for &(pressure, area) in &weighted {
        covered += area;
        if covered >= 0.05 * total {
            return pressure;
        }
    }
    f64::NAN
}

/// K5's readings (plan §15a): the peak push force, the largest of the
/// monitor's 100-step means of the axial contact force (plan §16i), and the
/// seated 95th-percentile pressure over μ.
fn k5_readings(model: &ExplicitModel, result: &TubeResult, mu: f64) -> String {
    let push_peak = result
        .samples
        .iter()
        .map(|s| s.monitors.contact_force[2])
        .fold(f64::NEG_INFINITY, f64::max);
    format!(
        "K5 push_peak={push_peak:.5}N seated_p95/mu={:.5}",
        seated_percentile(model, result) / mu
    )
}

/// The band's pair-averaged ring levels over μ (plan 15d.1), which K3
/// compares between precisions.
fn ring_levels(result: &TubeResult, mu: f64) -> String {
    result
        .reading
        .paired_levels
        .iter()
        .map(|p| format!("{:.6}", p / mu))
        .collect::<Vec<_>>()
        .join(",")
}

/// The steps, the last step, the rest step, and the loaded step factor: the
/// smallest step in the run over the rest step (plan §16i).
fn step_readings(result: &TubeResult) -> String {
    let rest = result.samples.first().map_or(f64::NAN, |s| s.dt);
    let smallest = result
        .samples
        .iter()
        .map(|s| s.dt)
        .fold(f64::INFINITY, f64::min);
    format!(
        "steps={} dt={:.3}us rest_dt={:.3}us loaded_factor={:.4}",
        result.steps,
        1e6 * result.dt,
        1e6 * rest,
        smallest / rest
    )
}

fn percent(x: f64) -> String {
    format!("{:+.2}%", 100.0 * x)
}

fn run_once(run: &TubeRun, wide: bool) -> TubeResult {
    if wide {
        run.run(|m, o| cpu::f64::CpuExecutor::new(m, o).unwrap())
    } else {
        run.run(|m, o| cpu::f32::CpuExecutor::new(m, o).unwrap())
    }
    .unwrap()
}

/// The Coulomb push (plan 15d.7), printed as its ratio and its two reactions:
/// the mandrel's axial push with friction, less the frictionless companion's,
/// over `μ_f Σ f_n`, each averaged over the constant-speed phase (10–90 % of
/// the loading time). K3 compares the reactions between precisions.
fn coulomb_push(run: &TubeRun, with_friction: &TubeResult, wide: bool) -> String {
    let frictionless = run_once(
        &TubeRun {
            friction: 0.0,
            ..*run
        },
        wide,
    );
    let loading = run.insertion.loading_time;
    let phase = |r: &TubeResult| {
        let inside: Vec<_> = r
            .samples
            .iter()
            .filter(|s| (0.1 * loading..=0.9 * loading).contains(&s.time))
            .map(|s| s.monitors)
            .collect();
        let count = inside.len() as f64;
        (
            inside.iter().map(|m| m.contact_force[2]).sum::<f64>() / count,
            inside.iter().map(|m| m.normal_force).sum::<f64>() / count,
        )
    };
    let ((push, normal), (geometric, _)) = (phase(with_friction), phase(&frictionless));
    format!(
        "{:.4}(push={push:.6}N,frictionless={geometric:.6}N)",
        (push - geometric) / (run.friction * normal)
    )
}

fn main() {
    let Request {
        run,
        case_index,
        wide,
    } = request();
    let case = run.case;
    let tube = Tube::plan(run.mesh);
    let model = tube.model(run.material(), case.walls).unwrap();
    let obstacle = run.obstacle(&tube).unwrap();
    let estimate = estimate_seconds(&model, &obstacle, wide);

    let started = Instant::now();
    let result = run_once(&run, wide);
    let wall = started.elapsed().as_secs_f64();
    let coulomb = (run.friction > 0.0 && case.walls == Walls::Free)
        .then(|| coulomb_push(&run, &result, wide));
    let flutter = result
        .samples
        .iter()
        .map(|s| s.monitors.contact_kinetic_energy)
        .fold(0.0, f64::max);

    let end = end_penetration(&run, &tube, &model, &obstacle, &result);
    let reached = result
        .samples
        .iter()
        .find(|s| s.monitors.max_penetration >= result.max_penetration)
        .map_or(f64::NAN, |s| s.time);
    let inset = (case.mandrel_ratio - 1.0) * tube.inner_radius;
    let stiffness = run.material().lambda + 2.0 * run.mu;
    let (k5, rings, stepping) = (
        k5_readings(&model, &result, run.mu),
        ring_levels(&result, run.mu),
        step_readings(&result),
    );
    let estimates = result.estimates as f64 * estimate;
    let missing = || "n/a".to_owned();
    println!(
        "tube {:?} case={case_index} ({}, a/A {}, nu {}) grid=A/{:.0} mu_f={} {} T={:.3}s mu={:.0}Pa threads={} | \
         h={:.3}mm p/(l+2mu)={:.5} inset={:.1}mm | \
         G2 grid_all_steps={:.1}um ({:.2}% of inset, first at t={reached:.3}s) true_end={:.1}um ({:.2}%) \
         grid_end={:.1}um bias(true-grid)=[{:.2},{:.2}]um deepest_z={:.2}mm | band_gap={:.1}um | \
         band_p/mu={:.6} K2 raw={} gc={} scatter={} rings/mu=[{rings}] | {k5} | \
         lz={} ke/ie={} balance={} inverted={} contact_ke_peak={:.3e}J coulomb={} | \
         {stepping} estimates={} wall={wall:.1}s ms/step={:.3} estimate={:.1}ms share={:.1}%",
        run.mesh,
        if case.walls == Walls::Free {
            "free"
        } else {
            "cased"
        },
        case.mandrel_ratio,
        case.poisson,
        tube.inner_radius / run.grid_cell,
        run.friction,
        if wide { "f64" } else { "f32" },
        run.insertion.loading_time,
        run.mu,
        std::env::var("RAYON_NUM_THREADS").unwrap_or_else(|_| "unset".to_owned()),
        1e3 * element_size(&model),
        case.pressure_over_mu * run.mu / stiffness,
        1e3 * inset,
        1e6 * result.max_penetration,
        100.0 * result.max_penetration / inset,
        1e6 * end.truth,
        100.0 * end.truth / inset,
        1e6 * end.grid,
        1e6 * end.bias.0,
        1e6 * end.bias.1,
        1e3 * end.deepest_z,
        1e6 * result.reading.gap,
        result.reading.pressure / run.mu,
        percent(result.errors.raw),
        result.errors.gap_corrected.map_or_else(missing, percent),
        percent(result.reading.node_scatter),
        percent(result.axial_stretch_error),
        result.kinetic_over_internal.map_or_else(missing, percent),
        result.energy_balance.map_or_else(missing, percent),
        result.inverted,
        flutter,
        coulomb.unwrap_or_else(missing),
        result.estimates,
        1e3 * (wall - estimates) / result.steps as f64,
        1e3 * estimate,
        100.0 * estimates / wall,
    );
}
