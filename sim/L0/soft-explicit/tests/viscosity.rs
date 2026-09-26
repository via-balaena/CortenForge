//! The material's Kelvin–Voigt viscosity (plan §16p): the stress against an
//! independent evaluation, what it must vanish for, its dissipation, its
//! place in the energy balance, the top mode's damping ratio against a dense
//! reference, and the damped stable step.

// The dense references name their matrices and sizes as the equations do
// (K, C, n, h), hence the single-character bindings.
#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::many_single_char_names
)]

mod common;

use common::{SILICONE, block_model, deform, displacements, gather};
use nalgebra::{DMatrix, Matrix3, SymmetricEigen};
use sim_soft_explicit::cpu;
use sim_soft_explicit::executor::{Executor, Obstacle};
use sim_soft_explicit::f64 as shared;
use sim_soft_explicit::f64::{Material, Pose, SdfGridLayout};
use sim_soft_explicit::stepping::{Stepper, StepperConfig, gates};
use sim_soft_explicit::{ExplicitModel, f32 as single};

/// A general displacement gradient, `det F ≈ 0.93`.
const H: [f64; 9] = [0.08, -0.12, 0.05, 0.03, -0.1, 0.09, -0.06, 0.04, -0.05];

/// A general rate `Ḟ`.
const RATE: [f64; 9] = [0.7, -1.3, 0.4, 2.1, 0.2, -0.9, -0.5, 1.1, -0.6];

fn matrix(m: [f64; 9]) -> Matrix3<f64> {
    Matrix3::from_row_slice(&m)
}

fn largest_entry(m: [f64; 9]) -> f64 {
    m.iter().fold(0.0, |a: f64, x| a.max(x.abs()))
}

#[test]
fn the_viscous_stress_is_twice_eta_times_the_deviatoric_stretching() {
    // P_v = J σ_v F⁻ᵀ with σ_v = 2η dev sym(Ḟ F⁻¹), by nalgebra's inverse.
    let eta = 7.0;
    let f = Matrix3::identity() + matrix(H);
    let l = matrix(RATE) * f.try_inverse().unwrap();
    let d = (l + l.transpose()) * 0.5;
    let deviator = d - Matrix3::identity() * (d.trace() / 3.0);
    let expected = deviator * (2.0 * eta) * f.determinant() * f.try_inverse().unwrap().transpose();
    let p = shared::first_piola_viscous(H, RATE, eta);
    let difference = (matrix(p) - expected).abs().max() / expected.abs().max();
    eprintln!("MARGIN viscous stress against nalgebra: {difference:e}");
    assert!(difference <= 1e-13, "{difference:e}");
}

#[test]
fn the_viscous_stress_vanishes_for_a_rigid_spin_and_a_pure_change_of_volume() {
    let f = Matrix3::identity() + matrix(H);
    let w = Matrix3::new(0.0, -0.8, 0.3, 0.8, 0.0, -1.2, -0.3, 1.2, 0.0);
    let spin: Vec<f64> = (w * f).transpose().iter().copied().collect();
    let spin: [f64; 9] = spin.try_into().unwrap();
    let swell: [f64; 9] = H.map(|x| 0.4 * x);
    let swell = {
        let mut m = swell;
        for i in [0, 4, 8] {
            m[i] += 0.4;
        }
        m
    };
    let scale = 2.0 * 7.0 * largest_entry(RATE);
    for (label, rate) in [("spin", spin), ("swell", swell)] {
        let p = shared::first_piola_viscous(H, rate, 7.0);
        let size = largest_entry(p) / scale;
        eprintln!("MARGIN viscous stress under a {label}: {size:e} of 2η|Ḟ|");
        assert!(size <= 1e-14, "{label}: {size:e}");
    }
    // A general rate gives a stress of order 2η|Ḟ|: the check can fail.
    assert!(largest_entry(shared::first_piola_viscous(H, RATE, 7.0)) > 0.1 * scale);
}

#[test]
fn viscous_forces_dissipate_and_their_power_is_the_stress_power() {
    let model = block_model(
        (2, 2, 2),
        0.01,
        Material {
            viscosity: 7.0,
            ..SILICONE
        },
    );
    let positions = deform(model.rest_positions(), 0.05);
    let u = displacements(&model, &positions);
    let v: Vec<[f64; 3]> = (0..model.node_count())
        .map(|a| {
            let x = a as f64;
            [
                0.03 * (1.3 * x).sin(),
                -0.02 * (0.7 * x).cos(),
                0.04 * (0.4 * x).sin(),
            ]
        })
        .collect();
    let mut total = 0.0;
    for (e, &element) in model.elements().iter().enumerate() {
        let (ue, ve) = (gather(&u, element), gather(&v, element));
        let (inverse, volume) = (model.rest_edge_inverses()[e], model.rest_volumes()[e]);
        let forces = shared::tet4_viscous_forces(ue, ve, inverse, volume, model.materials()[e]);
        let power: f64 = -forces.iter().zip(&ve).map(|(f, v)| f * v).sum::<f64>();
        // V P_v : Ḟ, with Ḟ the element's rate.
        let rate = shared::tet4_displacement_gradient(ve, inverse);
        let stress =
            shared::first_piola_viscous(shared::tet4_displacement_gradient(ue, inverse), rate, 7.0);
        let stress_power: f64 = volume * stress.iter().zip(&rate).map(|(p, r)| p * r).sum::<f64>();
        assert!(power >= -1e-18, "element {e}: {power:e}");
        assert!(
            (power - stress_power).abs() <= 1e-12 * stress_power.abs().max(1e-30),
            "element {e}: {power:e} against {stress_power:e}"
        );
        total += power;
    }
    assert!(total > 0.0);
}

#[test]
fn f32_viscous_forces_follow_f64() {
    let model = block_model((2, 2, 2), 0.01, SILICONE);
    let positions = deform(model.rest_positions(), 0.05);
    let u = displacements(&model, &positions);
    let v: Vec<[f64; 3]> = (0..model.node_count())
        .map(|a| [0.03 * (a as f64).sin(), -0.02, 0.01 * (a as f64).cos()])
        .collect();
    let narrow = |x: f64| x as f32;
    let material = single::Material {
        mu: narrow(SILICONE.mu),
        lambda: narrow(SILICONE.lambda),
        c2: 0.0,
        viscosity: 7.0,
        density: narrow(SILICONE.density),
    };
    let mut largest: f64 = 0.0;
    let mut difference: f64 = 0.0;
    for (e, &element) in model.elements().iter().enumerate() {
        let (ue, ve) = (gather(&u, element), gather(&v, element));
        let (inverse, volume) = (model.rest_edge_inverses()[e], model.rest_volumes()[e]);
        let wide = shared::tet4_viscous_forces(
            ue,
            ve,
            inverse,
            volume,
            Material {
                viscosity: 7.0,
                ..SILICONE
            },
        );
        let thin = single::tet4_viscous_forces(
            ue.map(narrow),
            ve.map(narrow),
            inverse.map(narrow),
            narrow(volume),
            material,
        );
        for (a, b) in wide.iter().zip(&thin) {
            largest = largest.max(a.abs());
            difference = difference.max((a - f64::from(*b)).abs());
        }
    }
    let relative = difference / largest;
    eprintln!("MARGIN f32 against f64 viscous forces: {relative:e}");
    assert!(relative <= 1e-5, "{relative:e}");
}

/// A block of 3 × 3 × 2 cubes of 10 mm in `material`, its top face held.
fn pressed_block(material: Material) -> ExplicitModel {
    let model = block_model((3, 3, 2), 0.01, material);
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

/// A floor at `z = 0` in its body frame, rising 1.5 mm into the block's
/// bottom face over 0.1 s while it slides sideways.
fn rising_floor(friction: f64) -> Obstacle {
    let (low, cell) = ([-0.01, -0.01, -0.005], 0.001);
    let size = [51, 51, 11];
    let grid = SdfGridLayout {
        origin_x: low[0],
        origin_y: low[1],
        origin_z: low[2],
        cell_size: cell,
        size_x: size[0],
        size_y: size[1],
        size_z: size[2],
    };
    let values = (0..size[2])
        .flat_map(|k| std::iter::repeat_n(low[2] + f64::from(k) * cell, 51 * 51))
        .collect();
    let poses = (0..=100)
        .map(|i| {
            let t = f64::from(i) * 0.001;
            Pose {
                qw: 1.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                tx: 0.005 * t,
                ty: 0.0,
                tz: -0.0005 + 0.02 * t,
            }
        })
        .collect();
    Obstacle {
        grid,
        values,
        start: 0.0,
        interval: 0.001,
        poses,
        friction,
    }
}

#[test]
fn the_energy_balance_holds_with_viscosity() {
    // No mass damping, so the viscosity is the only loss. At about 30 % of
    // the peak internal energy, the viscous work 5 % short moves the balance
    // by 1.5 %, past its 1 % (measured by that edit).
    let model = pressed_block(Material {
        viscosity: 300.0,
        ..SILICONE
    });
    let mut stepper = Stepper::new(
        cpu::f64::CpuExecutor::new(&model, &rising_floor(0.3)).unwrap(),
        StepperConfig::new(0.0),
        0.0,
    );
    stepper.run_until(0.1).unwrap();
    let samples = stepper.samples();
    let last = samples.last().unwrap().monitors;
    let peak = samples
        .iter()
        .map(|s| s.monitors.internal_energy)
        .fold(0.0, f64::max);
    eprintln!(
        "MARGIN viscous loss {:e} of peak internal energy",
        last.damping_loss / peak
    );
    assert!(
        last.damping_loss >= 0.25 * peak,
        "the viscous loss must be large enough to test"
    );
    let error = gates::energy_balance(samples).unwrap();
    eprintln!("MARGIN energy balance with viscosity: {error:e} of peak internal energy");
    assert!(error <= 0.01, "energy balance off by {error:e}");
}

/// The top eigenpair of `M⁻¹K` on the free nodes, by a dense eigensolve of
/// `M^(−½) K M^(−½)` with `K` from the executor's elastic forces.
fn dense_top_mode(model: &ExplicitModel, u: &[[f64; 3]]) -> (f64, Vec<[f64; 3]>) {
    let free: Vec<usize> = (0..model.node_count())
        .filter(|&a| !model.held()[a])
        .collect();
    let mut e = cpu::f64::CpuExecutor::new(model, &far_away()).unwrap();
    let zero = vec![[0.0; 3]; model.node_count()];
    let mut forces = |state: &[[f64; 3]]| {
        e.set_state(0.0, state, &zero, None);
        e.element_dilations();
        e.gather_volume_changes();
        e.nodal_pressures();
        e.element_forces();
        e.gather_forces();
        e.phase_outputs().elastic_forces
    };
    let n = 3 * free.len();
    let h = 1e-8;
    let mut k = DMatrix::<f64>::zeros(n, n);
    for (column, &a) in free.iter().enumerate() {
        for d in 0..3 {
            let (mut plus, mut minus) = (u.to_vec(), u.to_vec());
            plus[a][d] += h;
            minus[a][d] -= h;
            let (fp, fm) = (forces(&plus), forces(&minus));
            for (row, &b) in free.iter().enumerate() {
                for c in 0..3 {
                    k[(3 * row + c, 3 * column + d)] = -(fp[b][c] - fm[b][c]) / (2.0 * h);
                }
            }
        }
    }
    let k = (&k + k.transpose()) * 0.5;
    let root: Vec<f64> = free
        .iter()
        .flat_map(|&a| [model.node_masses()[a].sqrt(); 3])
        .collect();
    let scaled = DMatrix::from_fn(n, n, |i, j| k[(i, j)] / (root[i] * root[j]));
    let eigen = SymmetricEigen::new(scaled);
    let top = eigen.eigenvalues.imax();
    let mut mode = vec![[0.0; 3]; model.node_count()];
    for (row, &a) in free.iter().enumerate() {
        for c in 0..3 {
            mode[a][c] = eigen.eigenvectors[(3 * row + c, top)] / root[3 * row + c];
        }
    }
    (eigen.eigenvalues[top], mode)
}

/// A floor far below everything, never touched.
fn far_away() -> Obstacle {
    let mut obstacle = rising_floor(0.0);
    for pose in &mut obstacle.poses {
        pose.tz = -10.0;
    }
    obstacle
}

/// `vᵀCv / (2ω vᵀMv)`, with `C v` minus the executor's viscous forces at
/// velocities `v` in state `u`.
fn damping_ratio(model: &ExplicitModel, u: &[[f64; 3]], v: &[[f64; 3]], omega: f64) -> f64 {
    let mut e = cpu::f64::CpuExecutor::new(model, &far_away()).unwrap();
    e.set_state(0.0, u, v, None);
    e.element_forces();
    e.gather_forces();
    let viscous = e.phase_outputs().viscous_forces;
    let (mut dissipation, mut inertia) = (0.0, 0.0);
    for a in 0..model.node_count() {
        let m = model.node_masses()[a];
        for c in 0..3 {
            dissipation -= v[a][c] * viscous[a][c];
            inertia += m * v[a][c] * v[a][c];
        }
    }
    dissipation / (2.0 * omega * inertia)
}

#[test]
fn the_top_modes_damping_ratio_is_the_dense_ones() {
    let model = pressed_block(Material {
        viscosity: 20.0,
        ..SILICONE
    });
    let positions = deform(model.rest_positions(), 0.03);
    let u: Vec<[f64; 3]> = displacements(&model, &positions)
        .iter()
        .zip(model.held())
        .map(|(&u, &h)| if h { [0.0; 3] } else { u })
        .collect();
    let (omega_squared, mode) = dense_top_mode(&model, &u);
    let expected = damping_ratio(&model, &u, &mode, omega_squared.sqrt());
    let mut e = cpu::f64::CpuExecutor::new(&model, &far_away()).unwrap();
    e.set_state(0.0, &u, &vec![[0.0; 3]; model.node_count()], None);
    let p = e.epsilon().sqrt() * e.shortest_edge();
    let top = e.estimate_top_mode(StepperConfig::new(0.0).power_iterations, p, 0.0);
    let error = top.damping_ratio() / expected - 1.0;
    eprintln!(
        "MARGIN top mode's damping ratio: {:.4} against the dense {expected:.4} ({error:+e}); ω² {:+e}",
        top.damping_ratio(),
        top.omega_squared / omega_squared - 1.0
    );
    assert!(
        expected > 0.01,
        "the viscosity must be large enough to test"
    );
    assert!(error.abs() <= 0.05, "{error:+e}");
}

#[test]
fn a_highly_viscous_block_needs_the_damped_step_and_runs_on_it() {
    // The step's binding mode is a heavily damped one, far below the elastic
    // top mode, so a step from the elastic top mode would blow up; the loop's
    // damped step must not, at the start or at any re-estimate, which come
    // every 20 steps here so the run makes many.
    let model = pressed_block(Material {
        viscosity: 150.0,
        ..SILICONE
    });
    let mut stepper = Stepper::new(
        cpu::f64::CpuExecutor::new(&model, &rising_floor(0.0)).unwrap(),
        StepperConfig {
            reestimate_every: 20,
            ..StepperConfig::new(0.0)
        },
        0.0,
    );
    // The step the elastic top mode alone would give, damping included.
    let mut elastic = cpu::f64::CpuExecutor::new(&model, &rising_floor(0.0)).unwrap();
    let p = elastic.epsilon().sqrt() * elastic.shortest_edge();
    let top = elastic.estimate_top_mode(StepperConfig::new(0.0).power_iterations, p, 0.0);
    let from_elastic_top = StepperConfig::new(0.0).stable_step(top.omega_squared, top.damping);
    eprintln!(
        "MARGIN at 150 Pa·s: the elastic top mode's damping ratio {:.3}; the loop's step {:.3} of the step it gives",
        top.damping_ratio(),
        stepper.dt() / from_elastic_top
    );
    assert!(
        top.damping_ratio() > 0.3,
        "the viscosity must be large enough to test"
    );
    assert!(stepper.dt() < 0.8 * from_elastic_top);
    stepper.run_until(0.1).unwrap();
    assert!(
        stepper.estimates() > 10,
        "{} estimates",
        stepper.estimates()
    );
    assert!(stepper.samples().iter().all(|s| s.monitors.finite()));
    let error = gates::energy_balance(stepper.samples()).unwrap();
    assert!(error <= 0.01, "energy balance off by {error:e}");
}

/// The dense critical step of central differences with the damping force at
/// the lagging half-step velocity: the largest `Δt` at which
/// `M^(−½) (Δt² K + 2Δt C) M^(−½)` has no eigenvalue above 4, by bisection.
/// `K` and `C` are from the executor's elastic and viscous forces at rest.
fn dense_critical_step(model: &ExplicitModel) -> f64 {
    let free: Vec<usize> = (0..model.node_count())
        .filter(|&a| !model.held()[a])
        .collect();
    let n = 3 * free.len();
    let zero = vec![[0.0; 3]; model.node_count()];
    let mut e = cpu::f64::CpuExecutor::new(model, &far_away()).unwrap();
    let mut outputs = |u: &[[f64; 3]], v: &[[f64; 3]]| {
        e.set_state(0.0, u, v, None);
        e.element_dilations();
        e.gather_volume_changes();
        e.nodal_pressures();
        e.element_forces();
        e.gather_forces();
        e.phase_outputs()
    };
    let (h, mut k, mut c) = (
        1e-8,
        DMatrix::<f64>::zeros(n, n),
        DMatrix::<f64>::zeros(n, n),
    );
    for (column, &a) in free.iter().enumerate() {
        for d in 0..3 {
            let (mut plus, mut minus, mut unit) = (zero.clone(), zero.clone(), zero.clone());
            plus[a][d] = h;
            minus[a][d] = -h;
            unit[a][d] = 1.0;
            let (fp, fm) = (
                outputs(&plus, &zero).elastic_forces,
                outputs(&minus, &zero).elastic_forces,
            );
            let viscous = outputs(&zero, &unit).viscous_forces;
            for (row, &b) in free.iter().enumerate() {
                for r in 0..3 {
                    k[(3 * row + r, 3 * column + d)] = -(fp[b][r] - fm[b][r]) / (2.0 * h);
                    c[(3 * row + r, 3 * column + d)] = -viscous[b][r];
                }
            }
        }
    }
    let (k, c) = ((&k + k.transpose()) * 0.5, (&c + c.transpose()) * 0.5);
    let root: Vec<f64> = free
        .iter()
        .flat_map(|&a| [model.node_masses()[a].sqrt(); 3])
        .collect();
    let largest = |dt: f64| {
        let a = DMatrix::from_fn(n, n, |i, j| {
            (dt * dt * k[(i, j)] + 2.0 * dt * c[(i, j)]) / (root[i] * root[j])
        });
        SymmetricEigen::new(a).eigenvalues.max()
    };
    let (mut stable, mut unstable) = (0.0, 1.0);
    for _ in 0..64 {
        if largest(unstable) > 4.0 {
            break;
        }
        unstable *= 2.0;
    }
    for _ in 0..60 {
        let middle = 0.5 * (stable + unstable);
        if largest(middle) <= 4.0 {
            stable = middle;
        } else {
            unstable = middle;
        }
    }
    stable
}

#[test]
fn the_loops_step_is_the_safety_fraction_of_the_dense_critical_step() {
    for viscosity in [0.0, 20.0, 150.0] {
        let model = pressed_block(Material {
            viscosity,
            ..SILICONE
        });
        let critical = dense_critical_step(&model);
        let stepper = Stepper::new(
            cpu::f64::CpuExecutor::new(&model, &far_away()).unwrap(),
            StepperConfig::new(0.0),
            0.0,
        );
        let ratio = stepper.dt() / critical;
        eprintln!(
            "MARGIN loop's step over the dense critical step at {viscosity} Pa·s: {ratio:.4} (ξ {:.3})",
            stepper.damping() / (2.0 * stepper.omega_squared().sqrt())
        );
        assert!(
            (0.85..=0.9 * 1.02).contains(&ratio),
            "{viscosity} Pa·s: {ratio}"
        );
    }
}

#[test]
fn the_kinematic_prediction_carries_the_viscous_force() {
    // Every step ends with each node on or outside the rising, sliding floor
    // only if the prediction the contact undoes includes every force the
    // integration applies, the viscous one too (plan §16o's note on force
    // phases).
    let model = pressed_block(Material {
        viscosity: 20.0,
        ..SILICONE
    });
    let mut stepper = Stepper::new(
        cpu::f64::CpuExecutor::new(&model, &rising_floor(0.3)).unwrap(),
        StepperConfig::new(0.0),
        0.0,
    );
    stepper.run_until(0.1).unwrap();
    let last = stepper.samples().last().unwrap().monitors;
    eprintln!(
        "MARGIN deepest node with viscosity {:e} m (bar 1e-12 m)",
        last.max_penetration
    );
    assert!(last.contact_work > 0.0, "the floor must press");
    assert!(last.max_penetration <= 1e-12);
}
