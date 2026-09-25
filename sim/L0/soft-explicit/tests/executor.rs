//! The CPU executor and the stepping loop: one step against the reference
//! pipeline, constraints, the energy balance, precision, thread-count
//! independence, and the stable step's power iteration.

#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

mod common;

use common::{SILICONE, block_model, deform, displacements, elastic_forces};
use nalgebra::{DMatrix, SymmetricEigen};
use sim_soft_explicit::ExplicitModel;
use sim_soft_explicit::cpu;
use sim_soft_explicit::executor::{Executor, Obstacle, ObstacleError, Snapshot};
use sim_soft_explicit::f64 as shared;
use sim_soft_explicit::f64::{Pose, SdfGridLayout};
use sim_soft_explicit::stepping::{Stepper, StepperConfig, gates};

const IDENTITY: Pose = Pose {
    qw: 1.0,
    qx: 0.0,
    qy: 0.0,
    qz: 0.0,
    tx: 0.0,
    ty: 0.0,
    tz: 0.0,
};

/// A floor: in its body frame the surface is `z = 0` and the inside is
/// `z < 0`, baked at `cell` over `[low, high]`. Its pose track moves it by
/// `velocity` per second from `start` for `duration`.
fn floor(
    low: [f64; 3],
    high: [f64; 3],
    cell: f64,
    start: [f64; 3],
    velocity: [f64; 3],
    duration: f64,
    friction: f64,
) -> Obstacle {
    let size = |axis: usize| ((high[axis] - low[axis]) / cell).round() as u32 + 1;
    let grid = SdfGridLayout {
        origin_x: low[0],
        origin_y: low[1],
        origin_z: low[2],
        cell_size: cell,
        size_x: size(0),
        size_y: size(1),
        size_z: size(2),
    };
    let mut values = Vec::new();
    for k in 0..grid.size_z {
        for _ in 0..grid.size_y {
            for _ in 0..grid.size_x {
                values.push(low[2] + f64::from(k) * cell);
            }
        }
    }
    let samples = 101;
    let interval = duration / f64::from(samples - 1);
    let poses = (0..samples)
        .map(|i| {
            let t = f64::from(i) * interval;
            Pose {
                tx: start[0] + velocity[0] * t,
                ty: start[1] + velocity[1] * t,
                tz: start[2] + velocity[2] * t,
                ..IDENTITY
            }
        })
        .collect();
    Obstacle {
        grid,
        values,
        start: 0.0,
        interval,
        poses,
        friction,
        penalty_scale: 0.5,
    }
}

/// A floor far below everything, never touched.
fn nowhere() -> Obstacle {
    floor(
        [-1.0, -1.0, -1.0],
        [1.0, 1.0, 1.0],
        1.0,
        [0.0, 0.0, -10.0],
        [0.0; 3],
        1.0,
        0.0,
    )
}

/// A block of 3 × 3 × 2 cubes of 10 mm, its top face held.
fn pressed_block() -> ExplicitModel {
    let model = block_model((3, 3, 2), 0.01, SILICONE);
    let held: Vec<bool> = model
        .rest_positions()
        .iter()
        .map(|p| p[2] > 0.02 - 1e-9)
        .collect();
    ExplicitModel::new(
        model.rest_positions().to_vec(),
        model.elements().to_vec(),
        model.materials().to_vec(),
        held,
    )
    .unwrap()
}

/// A floor that rises into `pressed_block` by 1.5 mm and slides sideways.
fn rising_floor(friction: f64) -> Obstacle {
    floor(
        [-0.01, -0.01, -0.005],
        [0.04, 0.04, 0.005],
        0.001,
        [0.0, 0.0, -0.0005],
        [0.005, 0.0, 0.02],
        0.1,
        friction,
    )
}

fn run_phases(e: &mut impl Executor, time: f64, dt: f64, damping: f64) {
    e.element_dilations();
    e.gather_volume_changes();
    e.nodal_pressures();
    e.element_forces();
    e.gather_forces();
    e.contact(time, dt);
    e.integrate(dt, damping);
    e.boundary_conditions(dt, damping);
}

fn largest_difference(a: &[[f64; 3]], b: &[[f64; 3]]) -> f64 {
    a.iter()
        .flatten()
        .zip(b.iter().flatten())
        .fold(0.0, |m, (x, y)| m.max((x - y).abs()))
}

fn largest(a: &[[f64; 3]]) -> f64 {
    a.iter().flatten().fold(0.0, |m: f64, x| m.max(x.abs()))
}

#[test]
fn one_step_is_the_central_difference_update_of_the_reference_forces() {
    let model = block_model((3, 2, 2), 0.01, SILICONE);
    let positions = deform(model.rest_positions(), 0.05);
    let u = displacements(&model, &positions);
    let v: Vec<[f64; 3]> = (0..model.node_count())
        .map(|a| [0.01 * (a as f64).sin(), -0.02, 0.005 * (a as f64).cos()])
        .collect();
    let (dt, damping) = (1e-5, 30.0);
    let mut e = cpu::f64::CpuExecutor::new(&model, &nowhere()).unwrap();
    e.set_state(&u, &v);
    run_phases(&mut e, 0.0, dt, damping);
    let snapshot = e.snapshot();
    let forces = elastic_forces(&model, &positions);
    let expected: Vec<[f64; 3]> = (0..model.node_count())
        .map(|a| {
            shared::advance_velocity(v[a], forces[a], 1.0 / model.node_masses()[a], damping, dt)
        })
        .collect();
    let scale = largest(&expected);
    let error = largest_difference(&snapshot.velocities, &expected);
    assert!(
        error <= 1e-12 * scale,
        "velocity error {error:e} of {scale:e}"
    );
    let moved: Vec<[f64; 3]> = (0..model.node_count())
        .map(|a| shared::advance_displacement(u[a], expected[a], dt))
        .collect();
    assert!(largest_difference(&snapshot.displacements, &moved) <= 1e-15);
}

#[test]
fn held_and_constrained_directions_never_move() {
    let model = block_model((2, 2, 2), 0.01, SILICONE);
    let nodes = model.node_count();
    let held: Vec<bool> = (0..nodes).map(|a| a % 5 == 0).collect();
    let radial = [0.6, 0.8, 0.0];
    let constraints: Vec<[[f64; 3]; 2]> = (0..nodes)
        .map(|a| match a % 3 {
            0 => [radial, [0.0, 0.0, 1.0]],
            1 => [[1.0, 0.0, 0.0], [0.0; 3]],
            _ => [[0.0; 3]; 2],
        })
        .collect();
    let model = ExplicitModel::new(
        model.rest_positions().to_vec(),
        model.elements().to_vec(),
        model.materials().to_vec(),
        held.clone(),
    )
    .unwrap()
    .with_constraints(constraints.clone())
    .unwrap();
    let mut e = cpu::f64::CpuExecutor::new(&model, &nowhere()).unwrap();
    // A velocity kick in every direction; the first step removes what is not free.
    e.set_state(&vec![[0.0; 3]; nodes], &vec![[0.3, -0.2, 0.1]; nodes]);
    for step in 0..50 {
        run_phases(&mut e, f64::from(step) * 1e-5, 1e-5, 0.0);
    }
    let s = e.snapshot();
    for a in 0..nodes {
        let [first, second] = constraints[a];
        for (label, value) in [
            ("displacement", s.displacements[a]),
            ("velocity", s.velocities[a]),
        ] {
            if held[a] {
                assert_eq!(value, [0.0; 3], "held node {a}'s {label}");
            }
            for direction in [first, second] {
                assert!(
                    shared::vec3_dot(value, direction).abs() <= 1e-15,
                    "node {a}'s {label} along {direction:?}"
                );
            }
        }
    }
    assert!(largest(&s.displacements) > 0.0, "free directions must move");
}

fn pressed_run<E: Executor>(executor: E, friction: f64) -> Stepper<E> {
    let config = StepperConfig::new(0.5, friction, 50.0);
    let mut stepper = Stepper::new(executor, config, 0.0);
    stepper.run_until(0.1);
    stepper
}

#[test]
fn the_energy_balance_holds_with_contact_friction_and_damping() {
    let model = pressed_block();
    let stepper = pressed_run(
        cpu::f64::CpuExecutor::new(&model, &rising_floor(0.3)).unwrap(),
        0.3,
    );
    let samples = stepper.samples();
    let last = samples.last().unwrap().monitors;
    assert!(last.contact_work > 0.0 && last.damping_loss > 0.0);
    assert!(last.max_penetration > 0.0);
    assert!(!gates::inverted(samples));
    let error = gates::energy_balance(samples).unwrap();
    eprintln!(
        "MARGIN energy balance {error:e} of peak internal energy over {} steps",
        stepper.steps()
    );
    assert!(error <= 0.01, "energy balance off by {error:e}");
}

#[test]
fn f32_follows_f64_over_a_pressed_run() {
    let model = pressed_block();
    let wide = pressed_run(
        cpu::f64::CpuExecutor::new(&model, &rising_floor(0.3)).unwrap(),
        0.3,
    );
    let narrow = pressed_run(
        cpu::f32::CpuExecutor::new(&model, &rising_floor(0.3)).unwrap(),
        0.3,
    );
    assert_eq!(wide.steps(), narrow.steps());
    let (a, b) = (
        wide.executor().snapshot().displacements,
        narrow.executor().snapshot().displacements,
    );
    let relative = largest_difference(&a, &b) / largest(&a);
    eprintln!("MARGIN f32 against f64 displacement, pressed run: {relative:e}");
    assert!(relative <= 1e-3, "{relative:e}");
}

#[test]
fn the_result_does_not_depend_on_the_thread_count() {
    let model = pressed_block();
    let run = |threads: usize| -> Snapshot {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap();
        pool.install(|| {
            let e = cpu::f32::CpuExecutor::new(&model, &rising_floor(0.3)).unwrap();
            let config = StepperConfig::new(0.5, 0.3, 50.0);
            let mut stepper = Stepper::new(e, config, 0.0);
            stepper.run_until(0.03);
            stepper.executor().snapshot()
        })
    };
    let (one, many) = (run(1), run(4));
    assert!(largest(&one.displacements) > 0.0);
    assert_eq!(one, many, "one thread and four must agree bitwise");
}

/// `M^(−½) K M^(−½)`'s largest eigenvalue, with `K` the Hessian of the
/// reference pipeline's energy by central differences of its forces, over
/// the free nodes.
fn largest_generalized_eigenvalue(model: &ExplicitModel, positions: &[[f64; 3]]) -> f64 {
    let free: Vec<usize> = (0..model.node_count())
        .filter(|&a| !model.held()[a])
        .collect();
    let n = 3 * free.len();
    let h = 1e-7;
    let mut k = DMatrix::<f64>::zeros(n, n);
    for (column, &a) in free.iter().enumerate() {
        for d in 0..3 {
            let (mut plus, mut minus) = (positions.to_vec(), positions.to_vec());
            plus[a][d] += h;
            minus[a][d] -= h;
            let (fp, fm) = (elastic_forces(model, &plus), elastic_forces(model, &minus));
            for (row, &b) in free.iter().enumerate() {
                for c in 0..3 {
                    k[(3 * row + c, 3 * column + d)] = -(fp[b][c] - fm[b][c]) / (2.0 * h);
                }
            }
        }
    }
    let k = (&k + k.transpose()) * 0.5;
    let scale: Vec<f64> = free
        .iter()
        .flat_map(|&a| [1.0 / model.node_masses()[a].sqrt(); 3])
        .collect();
    let scaled = DMatrix::from_fn(n, n, |i, j| scale[i] * k[(i, j)] * scale[j]);
    SymmetricEigen::new(scaled)
        .eigenvalues
        .iter()
        .fold(f64::NEG_INFINITY, |m, &x| m.max(x))
}

#[test]
fn the_power_iteration_reaches_the_largest_eigenvalue_from_below() {
    let model = pressed_block();
    let positions = deform(model.rest_positions(), 0.03);
    let held: Vec<bool> = model.held().to_vec();
    let u: Vec<[f64; 3]> = displacements(&model, &positions)
        .iter()
        .zip(&held)
        .map(|(&u, &h)| if h { [0.0; 3] } else { u })
        .collect();
    let positions: Vec<[f64; 3]> = model
        .rest_positions()
        .iter()
        .zip(&u)
        .map(|(x, d)| [x[0] + d[0], x[1] + d[1], x[2] + d[2]])
        .collect();
    let exact = largest_generalized_eigenvalue(&model, &positions);
    let config = StepperConfig::new(0.5, 0.3, 0.0);
    let check = |e: &mut dyn Executor, label: &str| {
        e.set_state(&u, &vec![[0.0; 3]; model.node_count()]);
        let perturbation = e.epsilon().sqrt() * e.shortest_edge();
        let estimate = e.elastic_rayleigh_quotient(config.first_iterations, perturbation);
        let error = estimate / exact - 1.0;
        eprintln!("MARGIN power iteration ({label}): {error:+e} of ω_el²");
        assert!(error.abs() <= 0.05, "{label}: {error:+e}");
        assert!(
            estimate <= exact * (1.0 + 1e-3),
            "{label}: above the largest"
        );
    };
    check(
        &mut cpu::f64::CpuExecutor::new(&model, &nowhere()).unwrap(),
        "f64",
    );
    check(
        &mut cpu::f32::CpuExecutor::new(&model, &nowhere()).unwrap(),
        "f32",
    );
}

#[test]
fn the_stable_step_bounds_the_penalty_and_slipping_friction() {
    let omega_squared: f64 = 4.0e8;
    let omega = omega_squared.sqrt();
    let at =
        |friction: f64| StepperConfig::new(0.5, friction, 0.0).stable_step(omega_squared) * omega;
    assert!((at(0.0) - 2.74_f64.sqrt()).abs() < 1e-12);
    assert!((at(0.3) - 1.652).abs() < 5e-4, "{}", at(0.3));
    assert!((at(2.0) - 1.559).abs() < 5e-4, "{}", at(2.0));
    let no_budget = std::panic::catch_unwind(|| StepperConfig::new(3.3, 0.0, 0.0).stable_step(1.0));
    assert!(no_budget.is_err());
}

#[test]
fn a_window_sums_each_steps_contact_forces() {
    let model = pressed_block();
    let mut stepper = Stepper::new(
        cpu::f64::CpuExecutor::new(&model, &rising_floor(0.0)).unwrap(),
        StepperConfig::new(0.5, 0.0, 50.0),
        0.0,
    );
    stepper.run_until(0.09);
    stepper.open_window();
    for _ in 0..10 {
        stepper.step();
    }
    stepper.close_window();
    stepper.step();
    let s = stepper.executor().snapshot();
    assert_eq!(s.accumulated_steps, 10);
    let total: f64 = s.normal_force_sums.iter().sum();
    assert!(total > 0.0);
    assert!(
        s.friction_sums.iter().flatten().all(|&f| f == 0.0),
        "frictionless"
    );
}

#[test]
fn an_obstacle_is_checked_before_upload() {
    let model = block_model((1, 1, 1), 0.01, SILICONE);
    let base = nowhere();
    let cases: Vec<(Obstacle, fn(&ObstacleError) -> bool)> = vec![
        (
            Obstacle {
                poses: vec![],
                ..base.clone()
            },
            |e| matches!(e, ObstacleError::NoPoses),
        ),
        (
            Obstacle {
                interval: 0.0,
                ..base.clone()
            },
            |e| matches!(e, ObstacleError::Interval { .. }),
        ),
        (
            Obstacle {
                values: vec![0.0],
                ..base.clone()
            },
            |e| matches!(e, ObstacleError::GridSize { .. }),
        ),
        (
            Obstacle {
                friction: -0.1,
                ..base.clone()
            },
            |e| matches!(e, ObstacleError::Invalid { reason } if reason.contains("friction")),
        ),
        (
            Obstacle {
                penalty_scale: 0.0,
                ..base.clone()
            },
            |e| matches!(e, ObstacleError::Invalid { reason } if reason.contains("penalty")),
        ),
        (
            Obstacle {
                poses: vec![Pose {
                    qw: 2.0,
                    ..IDENTITY
                }],
                ..base
            },
            |e| matches!(e, ObstacleError::Invalid { reason } if reason.contains("unit")),
        ),
    ];
    for (obstacle, expected) in cases {
        let err = cpu::f64::CpuExecutor::new(&model, &obstacle).unwrap_err();
        assert!(expected(&err), "{err:?}");
    }
}
