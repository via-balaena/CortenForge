//! One run of the tube on the mandrel (plan §15b, §16i), printed as one line:
//! G2's penetration against the grid and the true surface, the band's gap,
//! K2's errors, the validity gates, and the run's cost.
//!
//! `cargo run --release -p sim-soft-explicit --example tube --
//! <10k|50k|100k> <case> <law> <friction> <f32|f64> <grid>`
//! (defaults: 10k 0 penalty:0.5 0 f32 20). `case` indexes
//! `fixtures::golden::THICK_TUBE`; `law` is `penalty:<s>`, `kinematic` or
//! `augmented:<s>:<steps between updates>`; the grid's cell is A/`grid`.
//! With friction on the free tube, a frictionless companion run gives the
//! Coulomb push ratio (plan 15d.7). Set `RAYON_NUM_THREADS` so the times are comparable.

#![allow(missing_docs, clippy::unwrap_used, clippy::cast_precision_loss)]

use std::time::Instant;

use sim_soft_explicit::ExplicitModel;
use sim_soft_explicit::cpu;
use sim_soft_explicit::executor::{ContactLaw, Executor, Obstacle};
use sim_soft_explicit::f64 as shared;
use sim_soft_explicit::fixtures::golden::THICK_TUBE;
use sim_soft_explicit::fixtures::tube::{Insertion, Mesh, Tube, TubeResult, TubeRun, Walls};
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
    let law_text = arg(2, "penalty:0.5");
    let parts: Vec<&str> = law_text.split(':').collect();
    let law = match parts.as_slice() {
        ["penalty", scale] => ContactLaw::Penalty {
            scale: scale.parse().unwrap(),
        },
        ["augmented", scale, interval] => ContactLaw::Augmented {
            scale: scale.parse().unwrap(),
            interval: interval.parse().unwrap(),
        },
        ["kinematic"] => ContactLaw::Kinematic,
        _ => {
            eprintln!("unknown law {law_text}: use penalty:<s>, kinematic or augmented:<s>:<n>");
            std::process::exit(2);
        }
    };
    let divisions: f64 = arg(5, "20").parse().unwrap();
    Request {
        run: TubeRun {
            mesh,
            case: THICK_TUBE[case_index],
            mu: MU,
            c2: 0.0,
            density: DENSITY,
            insertion: Insertion::plan(10.0 * TubeRun::shear_period(MU, DENSITY)),
            window: 0.1,
            friction: arg(3, "0").parse().unwrap(),
            law,
            grid_cell: Tube::plan(mesh).inner_radius / divisions,
        },
        case_index,
        wide: arg(4, "f32") == "f64",
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
    executor.elastic_rayleigh_quotient(iterations, perturbation);
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
    let grid_distance = |point: [f64; 3]| {
        let mut values = [0.0; 7];
        for (probe, value) in (0_u32..).zip(values.iter_mut()) {
            let coordinate = shared::sdf_probe_coordinate(point, obstacle.grid, probe);
            let corners = shared::sdf_cell_corners(coordinate, obstacle.grid)
                .map(|index| obstacle.values[index as usize]);
            *value = shared::sdf_trilinear(coordinate, corners);
        }
        shared::sdf_combine(values, obstacle.grid).distance
    };
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
        let (truth, grid) = (-mandrel.distance(body), -grid_distance(body));
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

/// The Coulomb push ratio (plan 15d.7): the mandrel's axial push with
/// friction, less the frictionless companion's, over `μ_f Σ f_n`, each
/// averaged over the constant-speed phase (10–90 % of the loading time).
fn coulomb_ratio(run: &TubeRun, with_friction: &TubeResult, wide: bool) -> f64 {
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
    (push - geometric) / (run.friction * normal)
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
        .then(|| coulomb_ratio(&run, &result, wide));
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
    let stiffness = run.material().lambda + 2.0 * MU;
    let estimates = result.estimates as f64 * estimate;
    let missing = || "n/a".to_owned();
    println!(
        "tube {:?} case={case_index} ({}, a/A {}, nu {}) law={:?} grid=A/{:.0} mu_f={} {} threads={} | \
         h={:.3}mm p/(l+2mu)={:.5} inset={:.1}mm | \
         G2 grid_all_steps={:.1}um ({:.2}% of inset, first at t={reached:.3}s) true_end={:.1}um ({:.2}%) \
         grid_end={:.1}um bias(true-grid)=[{:.2},{:.2}]um deepest_z={:.2}mm | band_gap={:.1}um | \
         K2 raw={} gc={} scatter={} | lz={} ke/ie={} balance={} inverted={} contact_ke_peak={:.3e}J coulomb={} | \
         steps={} dt={:.3}us estimates={} wall={wall:.1}s ms/step={:.3} estimate={:.1}ms share={:.1}%",
        run.mesh,
        if case.walls == Walls::Free {
            "free"
        } else {
            "cased"
        },
        case.mandrel_ratio,
        case.poisson,
        run.law,
        tube.inner_radius / run.grid_cell,
        run.friction,
        if wide { "f64" } else { "f32" },
        std::env::var("RAYON_NUM_THREADS").unwrap_or_else(|_| "unset".to_owned()),
        1e3 * element_size(&model),
        case.pressure_over_mu * MU / stiffness,
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
        percent(result.errors.raw),
        result.errors.gap_corrected.map_or_else(missing, percent),
        percent(result.reading.node_scatter),
        percent(result.axial_stretch_error),
        result.kinetic_over_internal.map_or_else(missing, percent),
        result.energy_balance.map_or_else(missing, percent),
        result.inverted,
        flutter,
        coulomb.map_or_else(missing, |c| format!("{c:.4}")),
        result.steps,
        1e6 * result.dt,
        result.estimates,
        1e3 * (wall - estimates) / result.steps as f64,
        1e3 * estimate,
        100.0 * estimates / wall,
    );
}
