//! The constitutive law, the element and averaged nodal pressure, checked
//! against definitions: stress is the energy's derivative, forces are the
//! mesh energy's gradient, and both are frame indifferent.

#![allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::cast_possible_truncation,
    clippy::many_single_char_names
)]

mod common;

use common::{SILICONE, block_model, deform, elastic_energy, elastic_forces};
use sim_soft_explicit::f64 as shared;
use sim_soft_explicit::f64::Material;

const YEOH: Material = Material {
    c2: 4.0e3,
    ..SILICONE
};

const IDENTITY: [f64; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];

/// A general deformation gradient with `det F ≈ 0.93`.
const F: [f64; 9] = [1.10, 0.20, -0.05, 0.03, 0.90, 0.10, -0.08, 0.04, 0.95];

fn max_abs(values: impl IntoIterator<Item = f64>) -> f64 {
    values.into_iter().fold(0.0, |m, v| m.max(v.abs()))
}

/// A rotation by `angle` about the unit axis `(1, 2, 2) / 3`.
fn rotation(angle: f64) -> [f64; 9] {
    let (s, c) = angle.sin_cos();
    let [x, y, z] = [1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0];
    let t = 1.0 - c;
    [
        t * x * x + c,
        t * x * y - s * z,
        t * x * z + s * y,
        t * x * y + s * z,
        t * y * y + c,
        t * y * z - s * x,
        t * x * z - s * y,
        t * y * z + s * x,
        t * z * z + c,
    ]
}

#[test]
fn the_rest_state_is_stress_and_energy_free() {
    for material in [SILICONE, YEOH] {
        assert!(max_abs(shared::first_piola(IDENTITY, material)) < 1e-12);
        assert_eq!(shared::energy_density(IDENTITY, material), 0.0);
    }
}

#[test]
fn the_stress_is_the_energys_derivative() {
    for material in [SILICONE, YEOH] {
        let p = shared::first_piola(F, material);
        for i in 0..9 {
            let h = 1e-6;
            let mut plus = F;
            let mut minus = F;
            plus[i] += h;
            minus[i] -= h;
            let derivative = (shared::energy_density(plus, material)
                - shared::energy_density(minus, material))
                / (2.0 * h);
            assert!(
                (derivative - p[i]).abs() <= 1e-6 * max_abs(p),
                "entry {i}: dΨ/dF {derivative} against P {}",
                p[i]
            );
        }
    }
}

#[test]
fn the_split_adds_up_to_the_full_law() {
    for material in [SILICONE, YEOH] {
        let j = shared::mat3_det(F);
        let lambda_term = shared::mat3_scale(
            shared::mat3_scale(shared::mat3_cofactor(F), 1.0 / j),
            material.lambda * j.ln(),
        );
        let split = shared::mat3_add(shared::first_piola_mu_terms(F, material), lambda_term);
        let full = shared::first_piola(F, material);
        assert!(max_abs((0..9).map(|i| split[i] - full[i])) <= 1e-12 * max_abs(full));
        let energy = shared::energy_density_mu_terms(F, material)
            + shared::energy_density_lambda_term(j, material.lambda);
        assert_eq!(energy, shared::energy_density(F, material));
        // U′(J) = λ ln J / J, per unit λ.
        let h = 1e-7;
        let derivative = (shared::energy_density_lambda_term(j + h, 1.0)
            - shared::energy_density_lambda_term(j - h, 1.0))
            / (2.0 * h);
        assert!((derivative - shared::pressure_per_lambda(j)).abs() < 1e-8);
    }
}

#[test]
fn uniaxial_stretch_matches_the_closed_form() {
    // F = diag(s, 1, 1): P₁₁ = μ(s − 1/s) + λ ln s / s + 4 C₂ (s² − 1) s.
    let s: f64 = 1.3;
    let f = [s, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    for material in [SILICONE, YEOH] {
        let expected = material.mu * (s - 1.0 / s)
            + material.lambda * s.ln() / s
            + 4.0 * material.c2 * (s * s - 1.0) * s;
        let p = shared::first_piola(f, material);
        assert!((p[0] - expected).abs() <= 1e-12 * expected.abs());
        assert!(max_abs([p[1], p[2], p[3], p[5], p[6], p[7]]) < 1e-9);
    }
}

#[test]
fn zero_c2_is_neo_hookean_bit_for_bit() {
    let neo_hookean = shared::first_piola(F, SILICONE);
    let expected: Vec<f64> = {
        let f_inverse_transpose =
            shared::mat3_scale(shared::mat3_cofactor(F), 1.0 / shared::mat3_det(F));
        (0..9)
            .map(|i| {
                SILICONE.mu * (F[i] - f_inverse_transpose[i])
                    + SILICONE.lambda * shared::mat3_det(F).ln() * f_inverse_transpose[i]
            })
            .collect()
    };
    assert!(max_abs((0..9).map(|i| neo_hookean[i] - expected[i])) <= 1e-12 * max_abs(neo_hookean));
    // The Yeoh term is added last, so C₂ = 0 changes nothing at all.
    let mu_terms = shared::first_piola_mu_terms(F, SILICONE);
    let f_inverse_transpose =
        shared::mat3_scale(shared::mat3_cofactor(F), 1.0 / shared::mat3_det(F));
    let plain: Vec<f64> = (0..9)
        .map(|i| SILICONE.mu * (F[i] - f_inverse_transpose[i]))
        .collect();
    assert_eq!(mu_terms.to_vec(), plain);
}

#[test]
fn the_stress_is_frame_indifferent() {
    // P(QF) = Q P(F) for a rotation Q.
    let q = rotation(0.7);
    for material in [SILICONE, YEOH] {
        let rotated = shared::first_piola(shared::mat3_mul(q, F), material);
        let expected = shared::mat3_mul(q, shared::first_piola(F, material));
        assert!(max_abs((0..9).map(|i| rotated[i] - expected[i])) <= 1e-10 * max_abs(expected));
    }
}

#[test]
fn inversion_is_reported_and_the_stress_stays_finite() {
    let inverted = [-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    assert!(shared::is_inverted(shared::mat3_det(inverted)));
    assert!(shared::is_inverted(0.0));
    assert!(!shared::is_inverted(1e-30));
    assert!(
        shared::first_piola(inverted, YEOH)
            .iter()
            .all(|v| v.is_finite())
    );
    assert!(shared::energy_density(inverted, YEOH).is_finite());
    assert!(shared::pressure_per_lambda(-0.5).is_finite());
}

#[test]
fn one_tets_forces_are_its_energys_gradient_and_balance() {
    let rest = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    assert_eq!(shared::tet4_volume(rest), 1.0 / 6.0);
    let edges = shared::tet4_edge_matrix(rest);
    let rest_inverse =
        shared::mat3_scale(shared::mat3_transpose(shared::mat3_cofactor(edges)), 1.0);
    let x = [
        0.05, -0.02, 0.01, 1.2, 0.1, -0.05, -0.1, 0.9, 0.08, 0.03, 0.12, 1.1,
    ];
    let energy = |x: [f64; 12]| shared::tet4_energy_mu_terms(x, rest_inverse, 1.0 / 6.0, YEOH);
    let forces = shared::tet4_elastic_forces(x, rest_inverse, 1.0 / 6.0, YEOH, 0.0);
    for i in 0..12 {
        let h = 1e-7;
        let mut plus = x;
        let mut minus = x;
        plus[i] += h;
        minus[i] -= h;
        let gradient = (energy(plus) - energy(minus)) / (2.0 * h);
        assert!(
            (forces[i] + gradient).abs() <= 1e-6 * max_abs(forces),
            "coordinate {i}"
        );
    }
    // Translation invariance: the forces sum to zero.
    for d in 0..3 {
        assert!(
            (forces[d] + forces[3 + d] + forces[6 + d] + forces[9 + d]).abs()
                < 1e-9 * max_abs(forces)
        );
    }
}

#[test]
fn the_pipelines_forces_are_the_mesh_energys_gradient() {
    // Plan §15c: with the λ term averaged over nodes, the force is the exact
    // gradient of Σ_e V_e Ψ_μ(F_e) + Σ_a V_a U(J_a).
    for material in [SILICONE, YEOH] {
        let model = block_model((2, 2, 2), 0.01, material);
        let deformed = deform(model.rest_positions(), 0.2);
        let forces = elastic_forces(&model, &deformed);
        let largest = max_abs(forces.iter().flatten().copied());
        let mut worst: f64 = 0.0;
        for node in 0..model.node_count() {
            for d in 0..3 {
                let h = 1e-9;
                let mut plus = deformed.clone();
                let mut minus = deformed.clone();
                plus[node][d] += h;
                minus[node][d] -= h;
                let gradient =
                    (elastic_energy(&model, &plus) - elastic_energy(&model, &minus)) / (2.0 * h);
                worst = worst.max((forces[node][d] + gradient).abs());
            }
        }
        assert!(
            worst <= 1e-6 * largest,
            "worst {worst:e} of largest force {largest:e}"
        );
    }
}

#[test]
fn the_rest_mesh_is_force_free_and_a_rotation_rotates_the_forces() {
    let model = block_model((2, 1, 2), 0.01, YEOH);
    let at_rest = elastic_forces(&model, model.rest_positions());
    assert!(max_abs(at_rest.iter().flatten().copied()) < 1e-12);

    let deformed = deform(model.rest_positions(), 0.2);
    let forces = elastic_forces(&model, &deformed);
    let q = rotation(1.1);
    let rotate = |p: [f64; 3]| {
        [
            q[0] * p[0] + q[1] * p[1] + q[2] * p[2],
            q[3] * p[0] + q[4] * p[1] + q[5] * p[2],
            q[6] * p[0] + q[7] * p[1] + q[8] * p[2],
        ]
    };
    let rotated: Vec<[f64; 3]> = deformed.iter().map(|&p| rotate(p)).collect();
    let rotated_forces = elastic_forces(&model, &rotated);
    let largest = max_abs(forces.iter().flatten().copied());
    for (got, f) in rotated_forces.iter().zip(&forces) {
        let expected = rotate(*f);
        assert!(max_abs((0..3).map(|d| got[d] - expected[d])) <= 1e-9 * largest);
    }
}

#[test]
fn where_materials_meet_each_element_keeps_its_own_pressure() {
    // Uniform compression: every node has the same J. Each element's
    // averaged pressure must be its own material's exact λ ln J / J, not a
    // blend with its neighbour's (IANP, plan §15g step 1).
    let stiff = Material {
        lambda: 10.0 * SILICONE.lambda,
        ..SILICONE
    };
    let (positions, elements) = common::block((2, 2, 2), 0.01);
    let materials: Vec<Material> = elements
        .iter()
        .map(|e| {
            if positions[e[0] as usize][0] < 0.005 {
                stiff
            } else {
                SILICONE
            }
        })
        .collect();
    let nodes = positions.len();
    let model =
        sim_soft_explicit::ExplicitModel::new(positions, elements, materials, vec![false; nodes])
            .unwrap();
    let scale: f64 = 0.97;
    let compressed: Vec<[f64; 3]> = model
        .rest_positions()
        .iter()
        .map(|p| p.map(|c| c * scale))
        .collect();
    let ratios = common::nodal_volume_ratios(&model, &compressed);
    let j = scale.powi(3);
    assert!(ratios.iter().all(|r| (r - j).abs() < 1e-12));
    let per_lambda: Vec<f64> = ratios
        .iter()
        .map(|&r| shared::pressure_per_lambda(r))
        .collect();
    let mut seen = [false; 2];
    for (e, element) in model.elements().iter().enumerate() {
        let lambda = model.materials()[e].lambda;
        let pressure = shared::element_pressure(lambda, element.map(|n| per_lambda[n as usize]));
        let exact = lambda * j.ln() / j;
        assert!(
            (pressure - exact).abs() <= 1e-12 * exact.abs(),
            "element {e}"
        );
        seen[usize::from(lambda == stiff.lambda)] = true;
    }
    assert_eq!(seen, [true, true], "both materials are present");
}

#[test]
fn f32_forces_agree_with_f64() {
    use sim_soft_explicit::f32 as single;
    let model = block_model((2, 2, 2), 0.01, YEOH);
    let deformed = deform(model.rest_positions(), 0.2);
    let reference = elastic_forces(&model, &deformed);
    let largest = max_abs(reference.iter().flatten().copied());

    // The same pipeline at f32, with every input rounded once.
    let narrow = |v: f64| v as f32;
    let material = single::Material {
        mu: narrow(YEOH.mu),
        lambda: narrow(YEOH.lambda),
        c2: narrow(YEOH.c2),
        density: narrow(YEOH.density),
    };
    let positions: Vec<[f32; 3]> = deformed.iter().map(|p| p.map(narrow)).collect();
    let gather = |e: [u32; 4]| {
        let mut x = [0.0_f32; 12];
        for (slot, &n) in e.iter().enumerate() {
            x[3 * slot..3 * slot + 3].copy_from_slice(&positions[n as usize]);
        }
        x
    };
    let mut current = vec![0.0_f32; model.node_count()];
    for &e in model.elements() {
        let v = single::tet4_volume(gather(e));
        for n in e {
            current[n as usize] += 0.25 * v;
        }
    }
    let per_lambda: Vec<f32> = current
        .iter()
        .zip(model.node_rest_volumes())
        .map(|(&v, &rest)| single::pressure_per_lambda(single::nodal_volume_ratio(v, narrow(rest))))
        .collect();
    let mut forces = vec![[0.0_f32; 3]; model.node_count()];
    for (i, &e) in model.elements().iter().enumerate() {
        let pressure = single::element_pressure(material.lambda, e.map(|n| per_lambda[n as usize]));
        let f = single::tet4_elastic_forces(
            gather(e),
            model.rest_edge_inverses()[i].map(narrow),
            narrow(model.rest_volumes()[i]),
            material,
            pressure,
        );
        for (slot, &n) in e.iter().enumerate() {
            for d in 0..3 {
                forces[n as usize][d] += f[3 * slot + d];
            }
        }
    }
    let worst = max_abs(
        forces
            .iter()
            .zip(&reference)
            .flat_map(|(a, b)| (0..3).map(move |d| f64::from(a[d]) - b[d])),
    );
    assert!(
        worst <= 1e-4 * largest,
        "f32 forces differ by {worst:e} of {largest:e}"
    );
}

#[test]
fn the_step_estimate_is_the_shortest_altitude_over_the_wave_speed() {
    let rest = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    // The slanted face has area √3/2, so the shortest altitude is 1/√3.
    assert!((shared::tet4_shortest_altitude(rest) - 1.0 / 3.0_f64.sqrt()).abs() < 1e-15);
    let speed = ((YEOH.lambda + 8.0 * YEOH.c2 + 2.0 * YEOH.mu) / YEOH.density).sqrt();
    assert_eq!(shared::dilatational_wave_speed(YEOH), speed);
    assert!(
        (shared::tet4_time_step_estimate(rest, YEOH) - 1.0 / 3.0_f64.sqrt() / speed).abs() < 1e-15
    );
}
