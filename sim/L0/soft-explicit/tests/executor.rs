//! The CPU executor and the stepping loop: each phase against the reference
//! pipeline, constraints, the friction anchors' state, the pose track, the
//! energy balance, precision, thread-count independence, the stable step,
//! and the stop on a non-finite read.

#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

mod common;

use common::{
    SILICONE, block_model, deform, displacements, elastic_forces, gather, nodal_dilations,
};
use nalgebra::{DMatrix, SymmetricEigen};
use sim_soft_explicit::ExplicitModel;
use sim_soft_explicit::cpu;
use sim_soft_explicit::executor::{
    Executor, Monitors, Obstacle, ObstacleError, Snapshot, rigid_motion,
};
use sim_soft_explicit::f64 as shared;
use sim_soft_explicit::f64::{Pose, SdfGridLayout};
use sim_soft_explicit::fixtures::grid::bricks;
use sim_soft_explicit::fixtures::tube::Mandrel;
use sim_soft_explicit::stepping::{RunError, Sample, Stepper, StepperConfig, gates};

const IDENTITY: Pose = Pose {
    qw: 1.0,
    qx: 0.0,
    qy: 0.0,
    qz: 0.0,
    tx: 0.0,
    ty: 0.0,
    tz: 0.0,
};

/// A pose track from `start`, moving by `velocity` per second, 101 samples
/// over `duration`.
fn track(start: [f64; 3], velocity: [f64; 3], duration: f64) -> (f64, Vec<Pose>) {
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
    (interval, poses)
}

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
    let (interval, poses) = track(start, velocity, duration);
    Obstacle {
        grid,
        values,
        fine: None,
        start: 0.0,
        interval,
        poses,
        friction,
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
    e.contact(time, dt, damping);
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

/// A deformed state of a 3 × 2 × 2 block, with a velocity field.
fn deformed_state(model: &ExplicitModel) -> (Vec<[f64; 3]>, Vec<[f64; 3]>, Vec<[f64; 3]>) {
    let positions = deform(model.rest_positions(), 0.05);
    let u = displacements(model, &positions);
    let v = (0..model.node_count())
        .map(|a| [0.01 * (a as f64).sin(), -0.02, 0.005 * (a as f64).cos()])
        .collect();
    (positions, u, v)
}

#[test]
fn one_step_is_the_central_difference_update_of_the_reference_forces() {
    let model = block_model((3, 2, 2), 0.01, SILICONE);
    let (positions, u, v) = deformed_state(&model);
    let (dt, damping) = (1e-5, 30.0);
    let mut e = cpu::f64::CpuExecutor::new(&model, &nowhere()).unwrap();
    e.set_state(0.0, &u, &v, None);
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
fn each_phase_matches_the_reference_pipeline() {
    let model = block_model((3, 2, 2), 0.01, SILICONE);
    let (positions, u, v) = deformed_state(&model);
    let mut e = cpu::f64::CpuExecutor::new(&model, &nowhere()).unwrap();
    e.set_state(0.0, &u, &v, None);
    run_phases(&mut e, 0.0, 1e-5, 0.0);
    let out = e.phase_outputs();
    for (element, (&d, rest_inverse)) in model
        .elements()
        .iter()
        .zip(out.dilations.iter().zip(model.rest_edge_inverses()))
    {
        let expected = shared::tet4_dilation(gather(&u, *element), *rest_inverse);
        assert!((d - expected).abs() <= 1e-15, "dilation {d} vs {expected}");
    }
    let nodal = nodal_dilations(&model, &positions);
    for (a, &dilation) in nodal.iter().enumerate() {
        let volume = model.node_rest_volumes()[a];
        assert!((out.volume_changes[a] / volume - dilation).abs() <= 1e-13);
        let pressure = shared::pressure_lambda_term(dilation, model.node_lambdas()[a]);
        assert!((out.pressures[a] - pressure).abs() <= 1e-9 * pressure.abs().max(1.0));
    }
    let forces = elastic_forces(&model, &positions);
    let scale = largest(&forces);
    assert!(largest_difference(&out.elastic_forces, &forces) <= 1e-12 * scale);
    // The element slots add up to the gathered forces.
    let mut summed = vec![[0.0; 3]; model.node_count()];
    for (element, f) in model.elements().iter().zip(&out.element_forces) {
        for (corner, &node) in element.iter().enumerate() {
            for d in 0..3 {
                summed[node as usize][d] += f[3 * corner + d];
            }
        }
    }
    assert!(largest_difference(&summed, &out.elastic_forces) <= 1e-12 * scale);
    assert!(out.contact_forces.iter().flatten().all(|&f| f == 0.0));
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
    // A displacement and velocity in every direction: set_state keeps only
    // the free parts, and every step after keeps them free.
    e.set_state(
        0.0,
        &vec![[1e-4, 2e-4, -1e-4]; nodes],
        &vec![[0.3, -0.2, 0.1]; nodes],
        None,
    );
    let check = |s: &Snapshot| {
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
    };
    check(&e.snapshot());
    for step in 0..50 {
        run_phases(&mut e, f64::from(step) * 1e-5, 1e-5, 0.0);
    }
    let s = e.snapshot();
    check(&s);
    assert!(
        largest(&s.displacements) > 1e-4,
        "free directions must move"
    );
}

/// A static floor 10 µm into `pressed_block`'s bottom from the start, its
/// frame translated by `shift` along its own plane.
fn touching_floor(shift: f64) -> Obstacle {
    floor(
        [-0.03, -0.01, -0.005],
        [0.04, 0.04, 0.005],
        0.001,
        [shift, 0.0, 1e-5],
        [0.0; 3],
        0.1,
        0.3,
    )
}

#[test]
fn moving_the_contact_frame_along_its_own_plane_changes_nothing() {
    // The anchors live in the obstacle's body frame. A node that starts in
    // contact must start anchored where it is in that frame, or the first
    // step pulls it with the full Coulomb force.
    let model = pressed_block();
    let run = |shift: f64| {
        let mut e = cpu::f64::CpuExecutor::new(&model, &touching_floor(shift)).unwrap();
        run_phases(&mut e, 0.0, 1e-5, 50.0);
        let first = e.phase_outputs();
        let sideways = first
            .contact_forces
            .iter()
            .map(|f| f[0].hypot(f[1]))
            .fold(0.0, f64::max);
        let normal: f64 = first.normal_forces.iter().sum();
        for step in 1..200 {
            run_phases(&mut e, f64::from(step) * 1e-5, 1e-5, 50.0);
        }
        (sideways, normal, e.snapshot().displacements)
    };
    let (sideways, normal, still) = run(0.0);
    let (sideways_shifted, _, shifted) = run(0.01);
    assert!(normal > 0.0, "the floor must touch from the start");
    assert!(sideways <= 1e-12 * normal && sideways_shifted <= 1e-12 * normal);
    let difference = largest_difference(&still, &shifted);
    assert!(
        difference <= 1e-9 * largest(&still),
        "{difference:e} of {:e}",
        largest(&still)
    );
}

#[test]
fn a_snapshot_puts_a_second_executor_in_the_same_state() {
    // Sliding frictional contact, so the anchors carry state: set_state from
    // a snapshot, anchors included, reproduces the next step bitwise.
    let model = pressed_block();
    let obstacle = rising_floor(0.3);
    let (dt, damping) = (2e-5, 50.0);
    let mut a = cpu::f64::CpuExecutor::new(&model, &obstacle).unwrap();
    for step in 0..3000 {
        run_phases(&mut a, f64::from(step) * dt, dt, damping);
    }
    let state = a.snapshot();
    assert!(largest(&state.anchors) > 0.0);
    let mut b = cpu::f64::CpuExecutor::new(&model, &obstacle).unwrap();
    let time = 3000.0 * dt;
    b.set_state(
        time,
        &state.displacements,
        &state.velocities,
        Some(&state.anchors),
    );
    run_phases(&mut a, time, dt, damping);
    run_phases(&mut b, time, dt, damping);
    let (after_a, after_b) = (a.snapshot(), b.snapshot());
    assert!(
        a.phase_outputs()
            .contact_forces
            .iter()
            .flatten()
            .any(|&f| f != 0.0)
    );
    assert_eq!(after_a.displacements, after_b.displacements);
    assert_eq!(after_a.velocities, after_b.velocities);
    assert_eq!(after_a.anchors, after_b.anchors);
}

#[test]
fn a_new_pose_track_takes_over_mid_run() {
    let model = pressed_block();
    let mut e = cpu::f64::CpuExecutor::new(&model, &rising_floor(0.0)).unwrap();
    let dt = 2e-5;
    for step in 0..2500 {
        run_phases(&mut e, f64::from(step) * dt, dt, 200.0);
    }
    // Pull the floor back out of reach: the contact must end.
    let (interval, poses) = track([0.0, 0.0, -0.01], [0.0; 3], 0.1);
    e.set_poses(2500.0 * dt, interval, &poses).unwrap();
    run_phases(&mut e, 2500.0 * dt, dt, 200.0);
    assert!(e.phase_outputs().normal_forces.iter().all(|&f| f == 0.0));
    assert!(matches!(
        e.set_poses(0.0, interval, &[]),
        Err(ObstacleError::NoPoses)
    ));
    assert!(matches!(
        e.set_poses(0.0, 0.0, &poses),
        Err(ObstacleError::Interval { .. })
    ));
    assert!(matches!(
        e.set_poses(f64::NAN, interval, &poses),
        Err(ObstacleError::Invalid { reason }) if reason.contains("start")
    ));
}

fn pressed_run<E: Executor>(executor: E, damping: f64) -> Stepper<E> {
    let mut stepper = Stepper::new(executor, StepperConfig::new(damping), 0.0);
    stepper.run_until(0.1).unwrap();
    stepper
}

#[test]
fn the_energy_balance_holds_with_contact_friction_and_damping() {
    // Heavily damped, so the damping term is a visible share of the balance:
    // at α 50 it is about the size of the balance's own error, and a doubled
    // damping term passed there.
    let model = pressed_block();
    let stepper = pressed_run(
        cpu::f64::CpuExecutor::new(&model, &rising_floor(0.3)).unwrap(),
        500.0,
    );
    let samples = stepper.samples();
    let last = samples.last().unwrap().monitors;
    assert!(last.contact_work > 0.0 && last.damping_loss > 0.0);
    assert!(!gates::inverted(samples));
    let peak = samples
        .iter()
        .map(|s| s.monitors.internal_energy)
        .fold(0.0, f64::max);
    eprintln!(
        "MARGIN damping loss {:e} and contact work {:e} of peak internal energy",
        last.damping_loss / peak,
        last.contact_work / peak
    );
    assert!(
        last.damping_loss >= 0.02 * peak,
        "the damping term must be large enough to test"
    );
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
    let mut wide = pressed_run(
        cpu::f64::CpuExecutor::new(&model, &rising_floor(0.3)).unwrap(),
        50.0,
    );
    let mut narrow = pressed_run(
        cpu::f32::CpuExecutor::new(&model, &rising_floor(0.3)).unwrap(),
        50.0,
    );
    assert_eq!(wide.steps(), narrow.steps());
    let (a, b) = (
        wide.executor_mut().snapshot().displacements,
        narrow.executor_mut().snapshot().displacements,
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
            assert_eq!(
                rayon::current_num_threads(),
                threads,
                "the run must use its pool"
            );
            let e = cpu::f32::CpuExecutor::new(&model, &rising_floor(0.3)).unwrap();
            let mut stepper = Stepper::new(e, StepperConfig::new(50.0), 0.0);
            stepper.run_until(0.03).unwrap();
            stepper.executor_mut().snapshot()
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
    let iterations = StepperConfig::new(0.0).power_iterations;
    let check = |e: &mut dyn Executor, label: &str| {
        e.set_state(0.0, &u, &vec![[0.0; 3]; model.node_count()], None);
        let perturbation = e.epsilon().sqrt() * e.shortest_edge();
        let estimate = e
            .estimate_top_mode(iterations, perturbation, 0.0)
            .omega_squared;
        let error = estimate / exact - 1.0;
        eprintln!("MARGIN power iteration ({label}, {iterations} iterations): {error:+e} of ω_el²");
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
fn the_stable_step_is_the_safety_fraction_of_the_damped_limit() {
    let config = StepperConfig::new(0.0);
    let omega_squared: f64 = 4.0e8;
    assert!((config.stable_step(omega_squared, 0.0) * omega_squared.sqrt() - 1.8).abs() < 1e-12);
    // Central differences' limit with the damping force at the lagging
    // half-step velocity: 2/ω (√(1 + ξ²) − ξ), here at ξ = 0.75, a damping
    // quotient of 2ξω (plan §16p).
    let omega = omega_squared.sqrt();
    assert!((config.stable_step(omega_squared, 1.5 * omega) * omega - 0.9).abs() < 1e-12);
    // Where the stiffness quotient is zero or negative, the damping alone
    // limits it: 2/γ at zero, and the root of ω²Δt² + 2γΔt = 4 below.
    assert!((config.stable_step(0.0, 1.0e4) * 1.0e4 - 1.8).abs() < 1e-12);
    let (negative, damping) = (-1.0e7, 1.0e4);
    let dt = config.stable_step(negative, damping) / 0.9;
    assert!((negative * dt * dt + 2.0 * damping * dt - 4.0).abs() < 1e-9);
    // With γ² + 4ω² < 0 the vector sets no limit, and no step is given.
    assert!(!config.stable_step(-1.0e9, damping).is_finite());
    // A smaller limit is taken at once; a larger one only 5 % at a time.
    assert_eq!(config.next_step(1.0, 0.8), 0.8);
    assert_eq!(config.next_step(1.0, 2.0), 1.05);
    assert_eq!(config.next_step(1.0, 1.02), 1.02);
}

#[test]
fn the_loop_re_estimates_its_step_as_it_runs() {
    let model = pressed_block();
    let config = StepperConfig {
        reestimate_every: 50,
        ..StepperConfig::new(50.0)
    };
    let mut stepper = Stepper::new(
        cpu::f64::CpuExecutor::new(&model, &rising_floor(0.3)).unwrap(),
        config,
        0.0,
    );
    stepper.run_until(0.1).unwrap();
    // Two at the start (`Stepper::estimate`), then one every 50 steps.
    let expected = 2 + (stepper.steps() - 1) / 50;
    assert_eq!(stepper.estimates(), expected);
    // The reads' steps never grow by more than 5 % from one read to the next.
    for pair in stepper.samples().windows(2) {
        assert!(pair[1].dt <= pair[0].dt * 1.05 * (1.0 + 1e-12));
    }
}

#[test]
fn an_estimate_depends_only_on_the_state() {
    // Each estimate starts from the same fixed vector: two at one state agree
    // bitwise. Warm-started from the last call instead, estimates stayed on a
    // lower mode once the tube was loaded (plan §16m).
    let model = pressed_block();
    let (_, u, v) = deformed_state(&model);
    let mut e = cpu::f64::CpuExecutor::new(&model, &nowhere()).unwrap();
    e.set_state(0.0, &u, &v, None);
    let p = e.epsilon().sqrt() * e.shortest_edge();
    let first = e.estimate_top_mode(40, p, 0.0);
    let second = e.estimate_top_mode(40, p, 0.0);
    assert_eq!(first, second);
}

#[test]
fn a_blow_up_at_a_re_estimate_stops_with_an_error() {
    // Re-estimating more often than reading: the blow-up reaches an estimate
    // first, which must return the error, not panic.
    let model = pressed_block();
    let config = StepperConfig {
        safety: 1.3,
        reestimate_every: 7,
        monitor_every: 100,
        ..StepperConfig::new(0.0)
    };
    let mut stepper = Stepper::new(
        cpu::f64::CpuExecutor::new(&model, &rising_floor(0.0)).unwrap(),
        config,
        0.0,
    );
    let result = stepper.run_until(0.1);
    assert!(
        matches!(result, Err(RunError::NonFinite { .. })),
        "{result:?}"
    );
}

#[test]
fn a_run_that_blows_up_stops_with_an_error() {
    // Past the stability limit (ω Δt = 2.6 > 2): the run must stop with an
    // error at the first non-finite read, not run on.
    let model = pressed_block();
    let config = StepperConfig {
        safety: 1.3,
        monitor_every: 10,
        ..StepperConfig::new(0.0)
    };
    let mut stepper = Stepper::new(
        cpu::f64::CpuExecutor::new(&model, &rising_floor(0.0)).unwrap(),
        config,
        0.0,
    );
    let result = stepper.run_until(0.1);
    assert!(
        matches!(result, Err(RunError::NonFinite { .. })),
        "{result:?}"
    );
    assert!(stepper.time() < 0.1, "it must stop early");
}

#[test]
fn the_gates_refuse_a_non_finite_read() {
    let good = Monitors {
        kinetic_energy: 1.0,
        internal_energy: 10.0,
        contact_work: 11.0,
        ..Monitors::default()
    };
    let bad = Monitors {
        kinetic_energy: f64::NAN,
        ..good
    };
    let sample = |monitors, step: u64| Sample {
        time: step as f64,
        step,
        dt: 1.0,
        monitors,
    };
    assert!(good.finite() && !bad.finite());
    // The moment and the obstacle's work are read too.
    let moment = Monitors {
        contact_moment: [0.0, f64::NAN, 0.0],
        ..good
    };
    let work = Monitors {
        obstacle_work: f64::INFINITY,
        ..good
    };
    assert!(!moment.finite() && !work.finite());
    let fine = [sample(good, 1), sample(good, 2)];
    let broken = [sample(good, 1), sample(bad, 2)];
    assert_eq!(gates::energy_balance(&fine), Some(0.0));
    assert_eq!(gates::energy_balance(&broken), None);
    assert!(gates::kinetic_over_internal(&fine, 0.0, 3.0).is_some());
    assert_eq!(gates::kinetic_over_internal(&broken, 0.0, 3.0), None);
}

#[test]
fn a_window_adds_each_steps_state_to_its_sums() {
    let model = pressed_block();
    let mut stepper = Stepper::new(
        cpu::f64::CpuExecutor::new(&model, &rising_floor(0.3)).unwrap(),
        StepperConfig::new(50.0),
        0.0,
    );
    stepper.run_until(0.09).unwrap();
    // The loop's window holds exactly the steps between opening and closing.
    stepper.open_window();
    for _ in 0..10 {
        stepper.step().unwrap();
    }
    stepper.close_window();
    stepper.step().unwrap();
    assert_eq!(stepper.executor_mut().snapshot().accumulated_steps, 10);
    // And the executor's sums add each call's state.
    let e = stepper.executor_mut();
    e.clear_accumulators();
    e.accumulate();
    let once = e.snapshot();
    e.accumulate();
    let twice = e.snapshot();
    assert_eq!(twice.accumulated_steps, 2);
    let total: f64 = once.normal_force_sums.iter().sum();
    assert!(total > 0.0, "the floor must be in contact");
    for (a, b) in once.normal_force_sums.iter().zip(&twice.normal_force_sums) {
        assert_eq!(2.0 * a, *b);
    }
    for (a, b) in once
        .friction_sums
        .iter()
        .flatten()
        .zip(twice.friction_sums.iter().flatten())
    {
        assert_eq!(2.0 * a, *b);
    }
    for (sum, u) in twice.displacement_sums.iter().zip(&once.displacements) {
        for d in 0..3 {
            assert_eq!(sum[d], 2.0 * u[d]);
        }
    }
    // The monitors' normal-force total is the nodes' sum, averaged per step.
    let mut e2 = cpu::f64::CpuExecutor::new(&model, &rising_floor(0.3)).unwrap();
    let snapshot = stepper.executor_mut().snapshot();
    e2.set_state(
        stepper.time(),
        &snapshot.displacements,
        &snapshot.velocities,
        Some(&snapshot.anchors),
    );
    let _ = e2.monitors();
    run_phases(&mut e2, stepper.time(), stepper.dt(), 50.0);
    let nodes: f64 = e2.phase_outputs().normal_forces.iter().sum();
    let read = e2.monitors();
    assert_eq!(read.steps, 1);
    assert!((read.normal_force - nodes).abs() <= 1e-12 * nodes);
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
                grid: SdfGridLayout {
                    size_x: u32::MAX,
                    size_y: u32::MAX,
                    size_z: u32::MAX,
                    ..base.grid
                },
                ..base.clone()
            },
            |e| matches!(e, ObstacleError::Invalid { reason } if reason.contains("u32 indexes")),
        ),
        (
            // Exactly u32::MAX samples: indexed to u32::MAX - 1, so accepted, and wrong only in its values' count.
            Obstacle {
                grid: SdfGridLayout {
                    size_x: 65_537,
                    size_y: 257,
                    size_z: 255,
                    ..base.grid
                },
                ..base.clone()
            },
            |e| matches!(e, ObstacleError::GridSize { expected, .. } if *expected == u32::MAX as usize),
        ),
        (
            // Past a u32, within a usize.
            Obstacle {
                grid: SdfGridLayout {
                    size_x: 1 << 16,
                    size_y: 1 << 16,
                    size_z: 2,
                    ..base.grid
                },
                ..base.clone()
            },
            |e| matches!(e, ObstacleError::Invalid { reason } if reason.contains("u32 indexes")),
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

/// One free node, pressed onto a cylinder by a compressed tetrahedron and
/// set sliding around it, frictionless and undamped. The kinematic law only
/// takes energy out, so the energy must not grow. Corrected along the normal
/// at the node's predicted position, which the inward push puts at a smaller
/// radius, each step carried the node further around than it went, and the
/// sliding grew by about 1 + a/R a step, `a` the push per step (plan §16o).
#[test]
fn kinematic_contact_does_not_feed_sliding_around_a_curved_obstacle() {
    let radius = 0.01;
    let z = -0.03; // on the mandrel's cylinder, behind its nose
    // The free node's rest position is 0.15 mm inside the cylinder, so held
    // on it the node is pushed in by about 0.46 mm a step, a/R ≈ 0.05, as on
    // the confined tube. It is node 1: the power iteration's fixed start has
    // no x component at node 0.
    let depth = 1.5e-4;
    let positions = vec![
        [radius + 0.004, -0.003, z - 0.002],
        [radius - depth, 0.0, z],
        [radius + 0.004, 0.003, z - 0.002],
        [radius + 0.004, 0.0, z + 0.003],
    ];
    let model = ExplicitModel::new(
        positions,
        vec![[0, 1, 3, 2]],
        vec![SILICONE],
        vec![true, false, true, true],
    )
    .unwrap();
    let (grid, values) = Mandrel { radius }
        .baked([-0.02, -0.02, -0.05], [0.02, 0.02, 0.0], 0.0005)
        .unwrap();
    let obstacle = Obstacle {
        grid,
        values,
        fine: None,
        start: 0.0,
        interval: 1.0,
        poses: vec![IDENTITY],
        friction: 0.0,
    };
    let mut executor = cpu::f64::CpuExecutor::new(&model, &obstacle).unwrap();
    // On the surface, sliding around it at 0.1 m/s.
    let mut u = vec![[0.0; 3]; 4];
    u[1] = [depth, 0.0, 0.0];
    let mut v = vec![[0.0; 3]; 4];
    v[1] = [0.0, 0.1, 0.0];
    executor.set_state(0.0, &u, &v, None);
    let energy = |m: Monitors| m.kinetic_energy + m.internal_energy;
    let start = energy(executor.monitors());
    let mut stepper = Stepper::new(executor, StepperConfig::new(0.0), 0.0);
    let mut largest = start;
    for _ in 0..400 {
        if stepper.step().is_err() {
            largest = f64::INFINITY;
            break;
        }
        largest = largest.max(energy(stepper.executor_mut().monitors()));
    }
    assert!(largest <= 1.01 * start, "energy {start:e} -> {largest:e}");
}

#[test]
fn the_kinematic_law_leaves_no_node_inside_a_moving_floor() {
    // G2, on the law's own terms: every step ends with each node on or
    // outside the floor, which rises and slides. Leftover depth would mean
    // the prediction used the floor where it was, not where it will be, or
    // missed the damping in the step it undoes.
    let model = pressed_block();
    let stepper = pressed_run(
        cpu::f64::CpuExecutor::new(&model, &rising_floor(0.3)).unwrap(),
        500.0,
    );
    let last = stepper.samples().last().unwrap().monitors;
    eprintln!(
        "MARGIN deepest node over the run {:e} m (bar 1e-12 m) after {} steps",
        last.max_penetration,
        stepper.steps()
    );
    assert!(last.contact_work > 0.0, "the floor must press");
    assert!(last.max_penetration <= 1e-12);
}

#[test]
fn the_executors_lookup_is_the_obstacles() {
    // One node 10 µm inside the mandrel, baked in a box that is off-centre and
    // longer in y than in x, so no two axes' values mirror each other: the
    // executor's own depth there is exactly `Obstacle::sample`'s.
    let radius = 0.011;
    let (angle, z) = (0.37_f64, -0.0312);
    let inside = [
        (radius - 1e-5) * angle.cos(),
        (radius - 1e-5) * angle.sin(),
        z,
    ];
    let positions = vec![
        inside,
        [0.02, 0.0, z],
        [0.02, 0.005, z - 0.004],
        [0.02, -0.005, z - 0.004],
    ];
    let positions = if shared::vec3_dot(
        shared::vec3_sub(positions[1], positions[0]),
        shared::vec3_cross(
            shared::vec3_sub(positions[2], positions[0]),
            shared::vec3_sub(positions[3], positions[0]),
        ),
    ) > 0.0
    {
        positions
    } else {
        vec![positions[0], positions[1], positions[3], positions[2]]
    };
    let model = ExplicitModel::new(
        positions,
        vec![[0, 1, 2, 3]],
        vec![SILICONE],
        vec![false, true, true, true],
    )
    .unwrap();
    let (grid, values) = Mandrel { radius }
        .baked([-0.022, -0.027, -0.05], [0.028, 0.028, 0.0], 0.0005)
        .unwrap();
    let obstacle = Obstacle {
        grid,
        values,
        fine: None,
        start: 0.0,
        interval: 1.0,
        poses: vec![IDENTITY],
        friction: 0.0,
    };
    let depth_in = |obstacle: &Obstacle| {
        let mut executor = cpu::f64::CpuExecutor::new(&model, obstacle).unwrap();
        run_phases(&mut executor, 0.0, 1e-6, 0.0);
        executor.monitors().max_penetration
    };
    let depth = depth_in(&obstacle);
    assert!(depth > 0.0);
    assert_eq!(depth, -obstacle.sample(inside).distance);

    // With a fine grid around the node, a quarter of the cell, it reads the fine grid; without the brick its
    // lookup needs, the coarse one.
    let (fine_grid, fine_values) = Mandrel { radius }
        .baked(
            inside.map(|x| x - 0.001),
            inside.map(|x| x + 0.001),
            0.000_25,
        )
        .unwrap();
    let fine = Obstacle {
        fine: Some(bricks(fine_grid, &fine_values, |_| true)),
        ..obstacle.clone()
    };
    let fine_depth = depth_in(&fine);
    assert_eq!(fine_depth, -fine.sample(inside).distance);
    assert_ne!(fine_depth, depth);
    let holed = Obstacle {
        fine: Some(bricks(fine_grid, &fine_values, |b| b != [0, 0, 0])),
        ..obstacle
    };
    assert_eq!(depth_in(&holed), depth);
}

#[test]
fn the_executors_fine_lookup_is_the_obstacles_on_and_off_its_bricks() {
    // One node inside the mandrel at each point, and the executor's depth there at both precisions against
    // `Obstacle::sample`: exactly at f64, within the f32 lookup's bar at f32 (0.1 µm, `sdf_lookup.rs`). The fine
    // grid, a quarter of the coarse cell, covers a box around the points; the points are inside its lattice, by a
    // face of it, just off it, and by a brick left out, so both grids answer somewhere.
    let radius = 0.011;
    let (grid, values) = Mandrel { radius }
        .baked([-0.022, -0.027, -0.05], [0.028, 0.028, 0.0], 0.0005)
        .unwrap();
    let coarse = Obstacle {
        grid,
        values,
        fine: None,
        start: 0.0,
        interval: 1.0,
        poses: vec![IDENTITY],
        friction: 0.0,
    };
    let (fine_grid, fine_values) = Mandrel { radius }
        .baked([0.006, 0.0, -0.034], [0.012, 0.006, -0.028], 0.000_25)
        .unwrap();
    let full = Obstacle {
        fine: Some(bricks(fine_grid, &fine_values, |_| true)),
        ..coarse.clone()
    };
    let holed = Obstacle {
        fine: Some(bricks(fine_grid, &fine_values, |b| b != [1, 1, 1])),
        ..coarse.clone()
    };
    let depth_in = |obstacle: &Obstacle, inside: [f64; 3]| {
        let z = inside[2];
        let mut positions = vec![
            inside,
            [0.02, 0.0, z],
            [0.02, 0.005, z - 0.004],
            [0.02, -0.005, z - 0.004],
        ];
        if shared::vec3_dot(
            shared::vec3_sub(positions[1], positions[0]),
            shared::vec3_cross(
                shared::vec3_sub(positions[2], positions[0]),
                shared::vec3_sub(positions[3], positions[0]),
            ),
        ) < 0.0
        {
            positions.swap(2, 3);
        }
        let model = ExplicitModel::new(
            positions,
            vec![[0, 1, 2, 3]],
            vec![SILICONE],
            vec![false, true, true, true],
        )
        .unwrap();
        let mut wide = cpu::f64::CpuExecutor::new(&model, obstacle).unwrap();
        run_phases(&mut wide, 0.0, 1e-6, 0.0);
        let mut narrow = cpu::f32::CpuExecutor::new(&model, obstacle).unwrap();
        run_phases(&mut narrow, 0.0, 1e-6, 0.0);
        (
            wide.monitors().max_penetration,
            narrow.monitors().max_penetration,
        )
    };
    // (obstacle, point, whether its fine grid answers there)
    let cases = [
        (&full, [0.0100, 0.0040, -0.0312], true),
        (&full, [0.0100, 0.0001, -0.0312], true),
        (&full, [0.0100, -0.0005, -0.0312], false),
        (&holed, [0.0090, 0.0030, -0.0310], false),
        (&holed, [0.0110 - 2e-4, 0.0010, -0.0330], true),
    ];
    for (obstacle, point, fine) in cases {
        let reads_fine = obstacle.fine.as_ref().unwrap().sample(point).is_some();
        assert_eq!(reads_fine, fine, "{point:?}");
        let expected = -obstacle.sample(point).distance;
        assert!(expected > 0.0, "{point:?} must be inside the mandrel");
        if fine {
            // The two grids read differently here, so the executor's answer tells which one it read.
            assert_ne!(expected, -coarse.sample(point).distance, "{point:?}");
        }
        let (wide, narrow) = depth_in(obstacle, point);
        assert_eq!(wide, expected, "{point:?}");
        assert!(
            (narrow - expected).abs() < 1e-7,
            "{point:?}: {narrow} {expected}"
        );
    }
}

/// The floor of [`rising_floor`], frictionless, with its values also in a fine grid of half its cell over the same
/// box, every brick kept: the fine grid answers every lookup the coarse one would.
fn rising_floor_with_a_fine_grid() -> Obstacle {
    let coarse = rising_floor(0.0);
    let fine = floor(
        [-0.01, -0.01, -0.005],
        [0.04, 0.04, 0.005],
        0.0005,
        [0.0; 3],
        [0.0; 3],
        1.0,
        0.0,
    );
    Obstacle {
        fine: Some(bricks(fine.grid, &fine.values, |_| true)),
        ..coarse
    }
}

/// `obstacle` pressed into [`pressed_block`], with one of its bottom nodes held too, for `steps` steps of a fixed
/// `dt` with no mass damping: the monitors at the end, the deepest correction any step made (each node's normal force
/// times `dt²` over its mass: the depth the kinematic law put it back by), and how many node-steps made one. The held
/// bottom node lies in the floor's way and takes no correction, so its depth must not be read.
fn corrections(obstacle: &Obstacle, steps: u32, dt: f64) -> (Monitors, f64, u64) {
    let block = pressed_block();
    let mut held = block.held().to_vec();
    let bottom = block
        .rest_positions()
        .iter()
        .position(|p| p[2] < 1e-9 && p[0] > 0.01 && p[1] > 0.01)
        .unwrap();
    held[bottom] = true;
    let model = ExplicitModel::new(
        block.rest_positions().to_vec(),
        block.elements().to_vec(),
        block.materials().to_vec(),
        held,
    )
    .unwrap();
    let mut executor = cpu::f64::CpuExecutor::new(&model, obstacle).unwrap();
    let (mut deepest, mut count) = (0.0_f64, 0_u64);
    for step in 0..steps {
        run_phases(&mut executor, f64::from(step) * dt, dt, 0.0);
        let outputs = executor.phase_outputs();
        for (force, mass) in outputs.normal_forces.iter().zip(model.node_masses()) {
            if *force > 0.0 {
                deepest = deepest.max(force * dt * dt / mass);
                count += 1;
            }
        }
    }
    (executor.monitors(), deepest, count)
}

#[test]
fn the_deepest_prediction_is_the_deepest_correction() {
    let (steps, dt) = (4_000, 2e-5);
    let (monitors, deepest, count) = corrections(&rising_floor(0.0), steps, dt);
    assert!(count > 1_000, "the floor must press: {count} node-steps");
    assert!(
        (monitors.deepest_prediction / deepest - 1.0).abs() < 1e-12,
        "{} against {deepest}",
        monitors.deepest_prediction
    );
    // The held node, which the floor passes through, is deeper than any prediction read.
    assert!(monitors.max_penetration > monitors.deepest_prediction);
}

#[test]
fn a_correction_the_coarse_grid_answered_is_counted() {
    let (steps, dt) = (4_000, 2e-5);
    // No fine grid: every correction read the coarse one.
    let (coarse, _, count) = corrections(&rising_floor(0.0), steps, dt);
    assert_eq!(coarse.coarse_corrections, count);
    // A fine grid over the whole box: none did, and the run is the same, the floor being linear.
    let (fine, deepest, fine_count) = corrections(&rising_floor_with_a_fine_grid(), steps, dt);
    assert_eq!(fine.coarse_corrections, 0);
    assert_eq!(fine_count, count);
    assert!((fine.deepest_prediction / deepest - 1.0).abs() < 1e-12);
}

/// A floor that starts turned 0.1 rad about x, then rises into [`pressed_block`] at 50 mm/s, slides along x at
/// 5 mm/s and turns about the world's y at `rate` rad/s, with friction 0.3. The turn starts from a turned pose, so its
/// rotation vector reads differently in the world frame and the body frame.
fn turning_floor(rate: f64) -> Obstacle {
    let samples = 101_u32;
    let duration = 0.1;
    let interval = duration / f64::from(samples - 1);
    let (tilt, centre) = (0.1_f64, 0.015);
    let poses = (0..samples)
        .map(|i| {
            let t = f64::from(i) * interval;
            let (c1, s1) = ((0.5 * rate * t).cos(), (0.5 * rate * t).sin());
            let (c2, s2) = ((0.5 * tilt).cos(), (0.5 * tilt).sin());
            // The turn about y after the tilt about x: q_y ⊗ q_x.
            Pose {
                qw: c1 * c2,
                qx: c1 * s2,
                qy: s1 * c2,
                qz: -s1 * s2,
                tx: centre + 0.005 * t,
                ty: centre,
                tz: -0.002 + 0.05 * t,
            }
        })
        .collect();
    Obstacle {
        start: 0.0,
        interval,
        poses,
        ..floor(
            [-0.04, -0.04, -0.04],
            [0.04, 0.04, 0.04],
            0.002,
            [0.0; 3],
            [0.0; 3],
            duration,
            0.3,
        )
    }
}

/// `steps` steps of `executor` against [`turning_floor`] at `dt`, after `lead` steps not read: the monitors read
/// after the lead, and for each read step each loaded node's position at the start of the step and its contact
/// force, from the state before the step and the phase outputs after it.
fn turning_steps<E: Executor>(
    executor: &mut E,
    model: &ExplicitModel,
    (lead, steps, dt): (u32, u32, f64),
) -> (Monitors, Vec<(f64, Vec<([f64; 3], [f64; 3])>)>) {
    for step in 0..lead {
        run_phases(executor, f64::from(step) * dt, dt, 0.0);
    }
    let after_lead = executor.monitors();
    let rest = model.rest_positions();
    let read = (lead..lead + steps)
        .map(|step| {
            let time = f64::from(step) * dt;
            let before = executor.snapshot().displacements;
            run_phases(executor, time, dt, 0.0);
            let forces = executor.phase_outputs().contact_forces;
            let loaded = forces
                .iter()
                .enumerate()
                .filter(|(_, f)| f.iter().any(|&c| c != 0.0))
                .map(|(a, f)| (shared::vec3_add(rest[a], before[a]), *f))
                .collect();
            (time, loaded)
        })
        .collect();
    (after_lead, read)
}

/// The mean over `steps` of `Σ (xᵢ − p) × fᵢ`, with `p` the obstacle's body origin as posed at `time + shift`.
fn mean_moment(
    obstacle: &Obstacle,
    steps: &[(f64, Vec<([f64; 3], [f64; 3])>)],
    shift: f64,
) -> [f64; 3] {
    let mut sum = [0.0; 3];
    for (time, loaded) in steps {
        let pose = obstacle.pose_at(time + shift);
        for &(at, force) in loaded {
            let arm = shared::vec3_sub(at, [pose.tx, pose.ty, pose.tz]);
            sum = shared::vec3_add(sum, shared::vec3_cross(arm, force));
        }
    }
    shared::vec3_scale(sum, 1.0 / steps.len() as f64)
}

#[test]
fn the_moment_is_taken_about_the_obstacles_origin_where_each_node_starts_its_step() {
    let (rate, dt) = (2.0, 2e-5);
    let obstacle = turning_floor(rate);
    let model = pressed_block();
    let schedule = (1_000, 2_000, dt);
    let check = |monitors: Monitors, steps: &[(f64, Vec<([f64; 3], [f64; 3])>)], bar: f64| {
        let loaded: usize = steps.iter().map(|(_, l)| l.len()).sum();
        assert!(loaded > 1_000, "the floor must press: {loaded} node-steps");
        let expected = mean_moment(&obstacle, steps, 0.0);
        let size = shared::vec3_length(expected);
        let error = shared::vec3_length(shared::vec3_sub(monitors.contact_moment, expected));
        assert!(
            error <= bar * size,
            "{:?} against {expected:?}",
            monitors.contact_moment
        );
        error / size
    };

    let mut wide = cpu::f64::CpuExecutor::new(&model, &obstacle).unwrap();
    let (_, steps) = turning_steps(&mut wide, &model, schedule);
    let error = check(wide.monitors(), &steps, 1e-12);
    // The case tells the reference point apart: the origin posed a step later reads far outside the bar.
    let later = mean_moment(&obstacle, &steps, dt);
    let expected = mean_moment(&obstacle, &steps, 0.0);
    let apart =
        shared::vec3_length(shared::vec3_sub(later, expected)) / shared::vec3_length(expected);
    assert!(apart > 1e3 * error.max(1e-15), "{apart}");

    let mut narrow = cpu::f32::CpuExecutor::new(&model, &obstacle).unwrap();
    let (_, steps) = turning_steps(&mut narrow, &model, schedule);
    check(narrow.monitors(), &steps, 1e-5);
}

#[test]
fn the_obstacles_work_is_its_rigid_motion_against_the_contact_forces() {
    let (rate, dt) = (2.0, 2e-5);
    let obstacle = turning_floor(rate);
    let model = pressed_block();
    let mut executor = cpu::f64::CpuExecutor::new(&model, &obstacle).unwrap();
    let (after_lead, steps) = turning_steps(&mut executor, &model, (1_000, 2_000, dt));
    let before = after_lead.obstacle_work;
    // The work each node's force does over the exact rigid motion of the obstacle's point there, and the same with
    // the turn taken in the body frame.
    let (mut exact, mut body_frame, mut largest_turn) = (0.0, 0.0, 0.0_f64);
    for (time, loaded) in &steps {
        let (from, to) = (obstacle.pose_at(*time), obstacle.pose_at(time + dt));
        let (moved, turn) = rigid_motion(from, to);
        largest_turn = largest_turn.max(shared::vec3_length(turn));
        let turned_back = shared::pose_unrotate(from, turn);
        for &(at, force) in loaded {
            let carried = shared::pose_to_world(to, shared::pose_to_body(from, at));
            exact += shared::vec3_dot(force, shared::vec3_sub(carried, at));
            let arm = shared::vec3_sub(at, [from.tx, from.ty, from.tz]);
            body_frame += shared::vec3_dot(
                force,
                shared::vec3_add(moved, shared::vec3_cross(turned_back, arm)),
            );
        }
    }
    let read = executor.monitors().obstacle_work - before;
    assert!(exact.abs() > 0.0, "the obstacle must do work");
    let bar = largest_turn;
    assert!(bar > 0.0 && bar < 1e-3, "{bar}");
    assert!(
        (read / exact - 1.0).abs() <= bar,
        "{read} against {exact} (bar {bar})"
    );
    // A turn taken in the body frame misses by far more than the bar.
    assert!(
        (body_frame / exact - 1.0).abs() > 10.0 * bar,
        "{body_frame} against {exact}"
    );
}

#[test]
fn a_rigid_motion_is_the_origins_move_and_the_world_frame_turn() {
    // Turned 0.3 rad about x, then a further 0.2 rad about the world's y, and moved.
    let about = |axis: [f64; 3], angle: f64| Pose {
        qw: (0.5 * angle).cos(),
        qx: axis[0] * (0.5 * angle).sin(),
        qy: axis[1] * (0.5 * angle).sin(),
        qz: axis[2] * (0.5 * angle).sin(),
        ..IDENTITY
    };
    let from = Pose {
        tx: 0.1,
        ty: -0.2,
        tz: 0.3,
        ..about([1.0, 0.0, 0.0], 0.3)
    };
    let turn = about([0.0, 1.0, 0.0], 0.2);
    // q_to = q_turn ⊗ q_from.
    let q = |p: Pose| (p.qw, [p.qx, p.qy, p.qz]);
    let ((w1, v1), (w2, v2)) = (q(turn), q(from));
    let v = shared::vec3_add(
        shared::vec3_add(shared::vec3_scale(v2, w1), shared::vec3_scale(v1, w2)),
        shared::vec3_cross(v1, v2),
    );
    let to = Pose {
        qw: w1 * w2 - shared::vec3_dot(v1, v2),
        qx: v[0],
        qy: v[1],
        qz: v[2],
        tx: 0.15,
        ty: -0.2,
        tz: 0.25,
    };
    let (moved, rotation) = rigid_motion(from, to);
    let close = |a: [f64; 3], b: [f64; 3]| (0..3).all(|i| (a[i] - b[i]).abs() < 1e-12);
    assert!(close(moved, [0.05, 0.0, -0.05]), "{moved:?}");
    assert!(close(rotation, [0.0, 0.2, 0.0]), "{rotation:?}");
    // The same turn with the quaternion's sign flipped, and no turn at all.
    let flipped = Pose {
        qw: -to.qw,
        qx: -to.qx,
        qy: -to.qy,
        qz: -to.qz,
        ..to
    };
    assert!(close(rigid_motion(from, flipped).1, [0.0, 0.2, 0.0]));
    assert_eq!(rigid_motion(from, from), ([0.0; 3], [0.0; 3]));
}

#[test]
fn a_new_pose_track_moves_the_moments_origin_and_a_still_obstacle_does_no_work() {
    let dt = 2e-5;
    let obstacle = turning_floor(2.0);
    let model = pressed_block();
    let mut executor = cpu::f64::CpuExecutor::new(&model, &obstacle).unwrap();
    for step in 0..1_500 {
        run_phases(&mut executor, f64::from(step) * dt, dt, 0.0);
    }
    // Held still from here, 1 mm along x from where the track had it.
    let time = 1_500.0 * dt;
    let now = obstacle.pose_at(time);
    let held = Pose {
        tx: now.tx + 0.001,
        ..now
    };
    executor.set_poses(time, 1.0, &[held]).unwrap();
    let before = executor.monitors();
    let rest = model.rest_positions();
    let steps: Vec<(f64, Vec<([f64; 3], [f64; 3])>)> = (1_500..2_000)
        .map(|step| {
            let displacements = executor.snapshot().displacements;
            run_phases(&mut executor, f64::from(step) * dt, dt, 0.0);
            let loaded = executor
                .phase_outputs()
                .contact_forces
                .iter()
                .enumerate()
                .filter(|(_, f)| f.iter().any(|&c| c != 0.0))
                .map(|(a, f)| (shared::vec3_add(rest[a], displacements[a]), *f))
                .collect();
            (f64::from(step) * dt, loaded)
        })
        .collect();
    let loaded: usize = steps.iter().map(|(_, l)| l.len()).sum();
    assert!(loaded > 1_000, "the floor must press: {loaded} node-steps");
    let still = Obstacle {
        start: time,
        interval: 1.0,
        poses: vec![held],
        ..obstacle
    };
    let read = executor.monitors();
    let expected = mean_moment(&still, &steps, 0.0);
    let error = shared::vec3_length(shared::vec3_sub(read.contact_moment, expected));
    assert!(
        error <= 1e-12 * shared::vec3_length(expected),
        "{:?} against {expected:?}",
        read.contact_moment
    );
    assert_eq!(read.obstacle_work, before.obstacle_work);
}

#[test]
fn f32_reads_the_obstacles_work_as_f64_far_from_the_origin() {
    // The block and the floor moved about 0.35 m from the origin, where f32 positions round to about 1e-8 m: the
    // executor takes the obstacle's move over a step from its pose track at f64, so its work follows f64's.
    let offset = [0.2, -0.15, 0.25];
    let block = pressed_block();
    let model = ExplicitModel::new(
        block
            .rest_positions()
            .iter()
            .map(|&p| shared::vec3_add(p, offset))
            .collect(),
        block.elements().to_vec(),
        block.materials().to_vec(),
        block.held().to_vec(),
    )
    .unwrap();
    let turning = turning_floor(2.0);
    let obstacle = Obstacle {
        poses: turning
            .poses
            .iter()
            .map(|p| Pose {
                tx: p.tx + offset[0],
                ty: p.ty + offset[1],
                tz: p.tz + offset[2],
                ..*p
            })
            .collect(),
        ..turning
    };
    let dt = 2e-5;
    let work = |executor: &mut dyn Executor| {
        for step in 0..4_000 {
            run_phases_dyn(executor, f64::from(step) * dt, dt);
        }
        executor.monitors().obstacle_work
    };
    let wide = work(&mut cpu::f64::CpuExecutor::new(&model, &obstacle).unwrap());
    let narrow = work(&mut cpu::f32::CpuExecutor::new(&model, &obstacle).unwrap());
    assert!(wide.abs() > 0.0);
    assert!(
        (narrow / wide - 1.0).abs() < 1e-4,
        "{narrow} against {wide}"
    );
}

fn run_phases_dyn(e: &mut dyn Executor, time: f64, dt: f64) {
    e.element_dilations();
    e.gather_volume_changes();
    e.nodal_pressures();
    e.element_forces();
    e.gather_forces();
    e.contact(time, dt, 0.0);
    e.integrate(dt, 0.0);
    e.boundary_conditions(dt, 0.0);
}
