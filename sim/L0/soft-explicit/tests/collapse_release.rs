//! The element collapsing on a public case (plan §16y): a block held at its
//! base and pressed by a rigid ball. The element as it was, selective ANP,
//! lets an element under the ball shrink below half its nodes' averaged
//! volume; the volumetric stabilization resists it, on every element or on
//! those alone. With the step re-estimated every 500 steps as the loop does,
//! the element as it was goes non-finite and stabilized everywhere it stands;
//! re-estimated every 50, the element as it was stands.
//!
//! Release-only: each run is some thousands of steps on 6144 elements.

#![allow(
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation
)]

mod common;

use std::collections::BTreeSet;
use std::f64::consts::TAU;

use common::block;
use sim_soft_explicit::ExplicitModel;
use sim_soft_explicit::cpu;
use sim_soft_explicit::executor::{Executor, Obstacle};
use sim_soft_explicit::f64::{Material, Pose};
use sim_soft_explicit::fixtures::grid::bake;
use sim_soft_explicit::fixtures::tube::ECOFLEX_00_30_VISCOUS_TIME;
use sim_soft_explicit::stepping::{Stepper, StepperConfig, gates};

/// Cubes of this side, 16 × 16 across and 4 deep.
const SIDE: f64 = 0.001;
const ACROSS: usize = 16;
const DEEP: usize = 4;
/// The ball's radius and how far it presses in.
const RADIUS: f64 = 0.003;
const DEPTH: f64 = 0.002;
const MU: f64 = 23.0e3;
const DENSITY: f64 = 1070.0;

/// Silicone at ν 0.49 with Ecoflex 00-30's viscosity (plan §16p).
fn material() -> Material {
    let nu = 0.49;
    Material {
        mu: MU,
        lambda: MU * 2.0 * nu / (1.0 - 2.0 * nu),
        c2: 0.0,
        viscosity: MU * ECOFLEX_00_30_VISCOUS_TIME,
        density: DENSITY,
    }
}

/// The block, its base held.
fn model() -> ExplicitModel {
    let (positions, elements) = block((ACROSS, ACROSS, DEEP), SIDE);
    let held = positions.iter().map(|p| p[2] < 1e-9).collect();
    let count = elements.len();
    ExplicitModel::new(positions, elements, vec![material(); count], held).unwrap()
}

/// The block's axial shear period, `4h/c_s`, the tube's rule (plan §15b).
fn shear_period() -> f64 {
    4.0 * DEEP as f64 * SIDE / (MU / DENSITY).sqrt()
}

/// The ball above the block's centre, pressed `DEPTH` in over 10 shear periods
/// on a smoothstep, then held 2; frictionless.
fn ball() -> Obstacle {
    let (width, height) = (ACROSS as f64 * SIDE, DEEP as f64 * SIDE);
    let cell = SIDE / 4.0;
    let half = ((width + 2.0 * RADIUS).max(height + 2.0 * RADIUS) / cell).ceil() * cell;
    let (grid, values) = bake([-half; 3], [half; 3], cell, |p| {
        (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt() - RADIUS
    })
    .unwrap();
    let (loading, hold) = (10.0 * shear_period(), 2.0 * shear_period());
    let samples: u32 = 401;
    let interval = (loading + hold) / f64::from(samples - 1);
    let start = height + RADIUS + 0.2 * SIDE;
    let travel = 0.2 * SIDE + DEPTH;
    let poses = (0..samples)
        .map(|i| {
            let s = (f64::from(i) * interval / loading).min(1.0);
            Pose {
                qw: 1.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                tx: width / 2.0,
                ty: width / 2.0,
                tz: start - travel * s * s * (3.0 - 2.0 * s),
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
        friction: 0.0,
    }
}

/// What a press shows of the collapse.
struct Press {
    /// The least element `J` over its nodes' averaged `J`, over every read.
    least: f64,
    /// The elements under half their nodes' averaged `J` at some read.
    collapsed: BTreeSet<usize>,
    /// The seated vertical contact force, at the last read.
    reaction: f64,
    finished: bool,
    inverted: bool,
    balance: Option<f64>,
}

/// Each element's `J` over the mean of its nodes' averaged `J`.
fn against_nodes(model: &ExplicitModel, dilations: &[f64], changes: &[f64]) -> Vec<f64> {
    let nodal: Vec<f64> = changes
        .iter()
        .zip(model.node_rest_volumes())
        .map(|(&change, &volume)| 1.0 + change / volume)
        .collect();
    model
        .elements()
        .iter()
        .zip(dilations)
        .map(|(nodes, &d)| {
            (1.0 + d) / (nodes.iter().map(|&n| nodal[n as usize]).sum::<f64>() / 4.0)
        })
        .collect()
}

/// One press of `model` at f32, the step re-estimated every `every` steps.
fn press(model: &ExplicitModel, every: u64) -> Press {
    let obstacle = ball();
    let config = StepperConfig {
        reestimate_every: every,
        ..StepperConfig::new(2.0 * 0.05 * TAU / shear_period())
    };
    let executor = cpu::f32::CpuExecutor::new(model, &obstacle).unwrap();
    let mut stepper = Stepper::new(executor, config, 0.0);
    let end = 12.0 * shear_period();
    let (mut least, mut collapsed, mut seen) = (f64::INFINITY, BTreeSet::new(), 0);
    let mut finished = true;
    loop {
        if stepper.time() >= end {
            break;
        }
        if stepper.step().is_err() {
            finished = false;
            break;
        }
        if stepper.samples().len() > seen {
            seen = stepper.samples().len();
            let outputs = stepper.executor_mut().phase_outputs();
            for (e, ratio) in against_nodes(model, &outputs.dilations, &outputs.volume_changes)
                .into_iter()
                .enumerate()
            {
                least = least.min(ratio);
                if ratio < 0.5 {
                    collapsed.insert(e);
                }
            }
        }
    }
    let samples = stepper.samples();
    Press {
        least,
        collapsed,
        reaction: samples
            .last()
            .map_or(f64::NAN, |s| s.monitors.contact_force[2]),
        finished,
        inverted: gates::inverted(samples),
        balance: gates::energy_balance(samples),
    }
}

#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "release-only: three presses of 6144 elements"
)]
fn an_element_collapses_under_the_ball_and_the_stabilization_resists_it() {
    let plain = model();
    let before = press(&plain, 50);
    eprintln!(
        "MARGIN as it was: least {:.3}, {} elements under half",
        before.least,
        before.collapsed.len()
    );
    assert!(before.finished && !before.inverted);
    assert!(before.balance.is_some_and(|b| b <= 0.01));
    assert!(before.least < 0.5 && !before.collapsed.is_empty());

    let everywhere = press(
        &plain.clone().with_volumetric_stabilization(2.0).unwrap(),
        50,
    );
    let kappa = 2.0 * MU;
    let masked = press(
        &plain
            .clone()
            .with_element_stabilizations(
                (0..plain.element_count())
                    .map(|e| {
                        if before.collapsed.contains(&e) {
                            kappa
                        } else {
                            0.0
                        }
                    })
                    .collect(),
            )
            .unwrap(),
        50,
    );
    for (label, after) in [("everywhere", everywhere), ("on those alone", masked)] {
        eprintln!(
            "MARGIN stabilized {label}: least {:.3}; the seated vertical contact force {:+.2} %",
            after.least,
            100.0 * (after.reaction / before.reaction - 1.0)
        );
        assert!(after.finished && !after.inverted, "{label}");
        assert!(after.least >= 0.5, "{label}: {}", after.least);
    }
}

#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "release-only: two presses of 6144 elements"
)]
fn the_element_as_it_was_does_not_stand_with_the_loops_re_estimate() {
    // Re-estimated every 500 steps, as the loop does, the element as it was
    // stops, not finite; re-estimated every 50 steps (the other test) it
    // stands (plan §16y rule 1). Stabilized everywhere at 2μ it finishes at
    // 500, uninverted and within the energy balance's bar.
    let every_500 = press(&model(), 500);
    eprintln!(
        "MARGIN as it was, every 500 steps: finished {}, inverted {}",
        every_500.finished, every_500.inverted
    );
    assert!(!every_500.finished);
    let stabilized = press(&model().with_volumetric_stabilization(2.0).unwrap(), 500);
    eprintln!(
        "MARGIN stabilized everywhere, every 500 steps: finished {}, inverted {}, least {:.3}",
        stabilized.finished, stabilized.inverted, stabilized.least
    );
    assert!(stabilized.finished && !stabilized.inverted);
    assert!(stabilized.balance.is_some_and(|b| b <= 0.01));
}
