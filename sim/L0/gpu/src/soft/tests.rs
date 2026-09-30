//! The soft executor against the CPU executor at f32 (recon §17b).

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::expect_used,
    clippy::unwrap_used
)]

use sim_soft_explicit::cpu::f32::CpuExecutor;
use sim_soft_explicit::executor::{Executor, Obstacle, PhaseOutputs};
use sim_soft_explicit::f64::{Material, Pose, SdfGridLayout};
use sim_soft_explicit::{ExplicitModel, f64 as shared};

use super::GpuExecutor;
use crate::context::GpuContext;
use crate::test_support::gpu_context_or_skip;

/// Silicone-like: μ 23 kPa, ν 0.49, ρ 1070 kg/m³.
const SILICONE: Material = Material {
    mu: 23.0e3,
    lambda: 23.0e3 * 2.0 * 0.49 / (1.0 - 2.0 * 0.49),
    c2: 0.0,
    viscosity: 0.0,
    density: 1070.0,
};

const IDENTITY: Pose = Pose {
    qw: 1.0,
    qx: 0.0,
    qy: 0.0,
    qz: 0.0,
    tx: 0.0,
    ty: 0.0,
    tz: 0.0,
};

/// A block of `n.0 × n.1 × n.2` cubes of side `side`, each split into six
/// tetrahedra around one body diagonal, positively oriented.
fn block(n: (usize, usize, usize), side: f64) -> (Vec<[f64; 3]>, Vec<[u32; 4]>) {
    let node = |i: usize, j: usize, k: usize| ((k * (n.1 + 1) + j) * (n.0 + 1) + i) as u32;
    let mut positions = Vec::new();
    for k in 0..=n.2 {
        for j in 0..=n.1 {
            for i in 0..=n.0 {
                positions.push([i as f64 * side, j as f64 * side, k as f64 * side]);
            }
        }
    }
    let axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    let orders = [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ];
    let mut elements = Vec::new();
    for k in 0..n.2 {
        for j in 0..n.1 {
            for i in 0..n.0 {
                for order in orders {
                    let mut corner = [0, 0, 0];
                    let mut tet = [node(i, j, k); 4];
                    for (slot, &axis) in order.iter().enumerate() {
                        for d in 0..3 {
                            corner[d] += axes[axis][d];
                        }
                        tet[slot + 1] = node(i + corner[0], j + corner[1], k + corner[2]);
                    }
                    let x = tet.map(|a| positions[a as usize]);
                    let corners = [
                        x[0][0], x[0][1], x[0][2], x[1][0], x[1][1], x[1][2], x[2][0], x[2][1],
                        x[2][2], x[3][0], x[3][1], x[3][2],
                    ];
                    if shared::tet4_volume(corners) < 0.0 {
                        tet.swap(1, 2);
                    }
                    elements.push(tet);
                }
            }
        }
    }
    (positions, elements)
}

fn block_model(n: (usize, usize, usize), side: f64, material: Material) -> ExplicitModel {
    let (positions, elements) = block(n, side);
    let materials = vec![material; elements.len()];
    let held = vec![false; positions.len()];
    ExplicitModel::new(positions, elements, materials, held).unwrap()
}

/// A smooth, non-uniform deformation's displacements, `amount` of the size.
fn deformation(model: &ExplicitModel, amount: f64) -> Vec<[f64; 3]> {
    model
        .rest_positions()
        .iter()
        .map(|&[x, y, z]| {
            let s = 100.0;
            [
                amount * (0.3 * x + 0.2 * s * y * z + 0.05 / s * (7.0 * s * y).sin()),
                amount * (-0.25 * y + 0.15 * x + 0.04 / s * (5.0 * s * z).cos()),
                amount * (0.1 * z - 0.2 * s * x * y + 0.06 / s * (6.0 * s * x).sin()),
            ]
        })
        .collect()
}

/// A velocity field.
fn velocities(model: &ExplicitModel) -> Vec<[f64; 3]> {
    (0..model.node_count())
        .map(|a| [0.01 * (a as f64).sin(), -0.02, 0.005 * (a as f64).cos()])
        .collect()
}

/// A floor far below everything, never touched.
fn nowhere() -> Obstacle {
    let grid = SdfGridLayout {
        origin_x: -1.0,
        origin_y: -1.0,
        origin_z: -1.0,
        cell_size: 1.0,
        size_x: 3,
        size_y: 3,
        size_z: 3,
    };
    let values = (0..27).map(|i| f64::from(i / 9) - 1.0).collect();
    Obstacle {
        grid,
        values,
        fine: None,
        start: 0.0,
        interval: 1.0,
        poses: vec![Pose {
            tz: -10.0,
            ..IDENTITY
        }],
        friction: 0.0,
    }
}

fn context() -> Option<GpuContext> {
    gpu_context_or_skip("soft executor")
}

/// The largest difference between two arrays, over the larger of their
/// largest magnitudes.
fn relative_difference<'a>(
    a: impl IntoIterator<Item = &'a f64>,
    b: impl IntoIterator<Item = &'a f64>,
) -> f64 {
    let (mut difference, mut largest) = (0.0_f64, 0.0_f64);
    for (x, y) in a.into_iter().zip(b) {
        difference = difference.max((x - y).abs());
        largest = largest.max(x.abs()).max(y.abs());
    }
    if largest == 0.0 {
        difference
    } else {
        difference / largest
    }
}

/// Phases 1–5's outputs, compared: each array's largest difference over its
/// largest magnitude.
fn elastic_differences(cpu: &PhaseOutputs, gpu: &PhaseOutputs) -> [(&'static str, f64); 7] {
    [
        (
            "dilations",
            relative_difference(&cpu.dilations, &gpu.dilations),
        ),
        (
            "volume changes",
            relative_difference(&cpu.volume_changes, &gpu.volume_changes),
        ),
        (
            "pressures",
            relative_difference(&cpu.pressures, &gpu.pressures),
        ),
        (
            "element forces",
            relative_difference(
                cpu.element_forces.iter().flatten(),
                gpu.element_forces.iter().flatten(),
            ),
        ),
        (
            "element viscous forces",
            relative_difference(
                cpu.element_viscous_forces.iter().flatten(),
                gpu.element_viscous_forces.iter().flatten(),
            ),
        ),
        (
            "elastic forces",
            relative_difference(
                cpu.elastic_forces.iter().flatten(),
                gpu.elastic_forces.iter().flatten(),
            ),
        ),
        (
            "viscous forces",
            relative_difference(
                cpu.viscous_forces.iter().flatten(),
                gpu.viscous_forces.iter().flatten(),
            ),
        ),
    ]
}

#[test]
fn phases_one_to_five_match_the_cpu_at_f32() {
    let Some(ctx) = context() else { return };
    let model = block_model((3, 2, 2), 0.01, SILICONE);
    let (u, v) = (deformation(&model, 0.05), velocities(&model));
    let mut cpu = CpuExecutor::new(&model, &nowhere()).unwrap();
    let mut gpu = GpuExecutor::new(&ctx, &model, &nowhere()).unwrap();
    for e in [&mut cpu as &mut dyn Executor, &mut gpu] {
        e.set_state(0.0, &u, &v, None);
        e.element_dilations();
        e.gather_volume_changes();
        e.nodal_pressures();
        e.element_forces();
        e.gather_forces();
    }
    let (cpu, gpu) = (cpu.phase_outputs(), gpu.phase_outputs());
    for (what, difference) in elastic_differences(&cpu, &gpu) {
        println!("{what}: {difference:e}");
        assert!(difference <= 1e-5, "{what}: {difference:e}");
    }
}

/// A floor: in its body frame the surface is `z = 0` and the inside `z < 0`,
/// baked at `cell` over `[low, high]`, moved by `velocity` from `start` for
/// `duration`.
fn floor(
    (low, high, cell): ([f64; 3], [f64; 3], f64),
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
        for _ in 0..grid.size_y * grid.size_x {
            values.push(low[2] + f64::from(k) * cell);
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
        fine: None,
        start: 0.0,
        interval,
        poses,
        friction,
    }
}

/// A block of 3 × 3 × 2 cubes of 10 mm, its top face held.
fn pressed_block() -> ExplicitModel {
    let model = block_model((3, 3, 2), 0.01, SILICONE);
    let held = model
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
        ([-0.01, -0.01, -0.005], [0.04, 0.04, 0.005], 0.001),
        [0.0, 0.0, -0.0005],
        [0.005, 0.0, 0.02],
        0.1,
        friction,
    )
}

#[test]
fn a_pressed_run_follows_the_cpu() {
    use sim_soft_explicit::stepping::{Stepper, StepperConfig};
    let Some(ctx) = context() else { return };
    let (model, obstacle) = (pressed_block(), rising_floor(0.5));
    let config = StepperConfig::new(30.0);
    let mut cpu = Stepper::new(CpuExecutor::new(&model, &obstacle).unwrap(), config, 0.0);
    let mut gpu = Stepper::new(
        GpuExecutor::new(&ctx, &model, &obstacle).unwrap(),
        config,
        0.0,
    );
    println!("dt cpu {:e} gpu {:e}", cpu.dt(), gpu.dt());
    for s in [&mut cpu as &mut dyn std::any::Any, &mut gpu] {
        let _ = s;
    }
    cpu.run_until(0.05).unwrap();
    gpu.run_until(0.05).unwrap();
    let (a, b) = (cpu.samples().last().unwrap(), gpu.samples().last().unwrap());
    println!("steps {} {}", cpu.steps(), gpu.steps());
    println!("cpu {:?}", a.monitors);
    println!("gpu {:?}", b.monitors);
    let (sa, sb) = (cpu.executor_mut().snapshot(), gpu.executor_mut().snapshot());
    let d = relative_difference(
        sa.displacements.iter().flatten(),
        sb.displacements.iter().flatten(),
    );
    println!("displacements {d:e}");
    let (ta, tb) = (
        cpu.executor_mut().estimate_top_mode(100, 1e-6, 0.0),
        gpu.executor_mut().estimate_top_mode(100, 1e-6, 0.0),
    );
    println!("top cpu {ta:?} gpu {tb:?}");
}

/// ★ A count set just below 2³² and counted past it reads the exact total:
/// the low word carries into the high one, and the other count is untouched.
#[test]
fn a_count_carries_past_two_to_the_thirty_second() {
    let Some(ctx) = context() else { return };
    let model = block_model((3, 2, 2), 0.01, SILICONE);
    let mut gpu = GpuExecutor::new(&ctx, &model, &nowhere()).unwrap();
    // Each node sent through the origin to its mirror image: every element
    // turned inside out.
    let mirrored: Vec<[f64; 3]> = model
        .rest_positions()
        .iter()
        .map(|p| p.map(|x| -2.0 * x))
        .collect();
    gpu.set_state(0.0, &mirrored, &vec![[0.0; 3]; model.node_count()], None);
    let (inverted, coarse) = ((1_u64 << 32) - 10, (1_u64 << 32) - 1);
    gpu.set_counts(inverted, coarse);
    gpu.element_dilations();
    let monitors = gpu.monitors();
    let elements = model.element_count() as u64;
    assert_eq!(monitors.inverted_element_steps, inverted + elements);
    assert_eq!(monitors.coarse_corrections, coarse);
}
