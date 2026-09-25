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

use common::{SILICONE, block_model, deform, elastic_forces};
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
        assert!((derivative - shared::pressure_lambda_term(j, 1.0)).abs() < 1e-8);
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
    assert!(shared::pressure_lambda_term(-0.5, YEOH.lambda).is_finite());
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

/// Two materials, split across the block, so some nodes sit where they meet.
fn two_material_model() -> sim_soft_explicit::ExplicitModel {
    let stiff = Material {
        mu: 3.0 * SILICONE.mu,
        lambda: 10.0 * SILICONE.lambda,
        ..YEOH
    };
    let (positions, elements) = common::block((2, 2, 2), 0.01);
    let materials: Vec<Material> = elements
        .iter()
        .map(|e| {
            let centroid_x: f64 = e.iter().map(|&n| positions[n as usize][0]).sum::<f64>() / 4.0;
            if centroid_x < 0.01 { stiff } else { SILICONE }
        })
        .collect();
    let nodes = positions.len();
    sim_soft_explicit::ExplicitModel::new(positions, elements, materials, vec![false; nodes])
        .unwrap()
}

#[test]
fn the_pipelines_forces_are_the_mesh_energys_gradient() {
    // Plan §15c: with the λ term averaged over nodes, the force is the exact
    // gradient of Σ_e V_e Ψ_μ(F_e) + Σ_a V_a λ_a/2 (ln J_a)², in one
    // material or two.
    let models = [
        ("neo-Hookean", block_model((2, 2, 2), 0.01, SILICONE)),
        ("Yeoh", block_model((2, 2, 2), 0.01, YEOH)),
        ("two materials", two_material_model()),
    ];
    for (name, model) in models {
        let deformed = deform(model.rest_positions(), 0.2);
        let (worst, largest) = common::gradient_error(&model, &deformed, 1e-9);
        eprintln!("MARGIN gradient ({name}): {worst:e} of largest force {largest:e}");
        assert!(worst <= 1e-7 * largest, "{name}: {worst:e} of {largest:e}");
    }
}

#[test]
fn a_nodes_lambda_blends_only_where_materials_meet() {
    let model = two_material_model();
    let (soft, stiff) = (SILICONE.lambda, 10.0 * SILICONE.lambda);
    let mut blended = 0;
    for node in 0..model.node_count() {
        // The rest-volume-weighted λ of the elements around the node.
        let (mut weighted, mut volume) = (0.0, 0.0);
        for (e, element) in model.elements().iter().enumerate() {
            if element.contains(&(node as u32)) {
                weighted += model.rest_volumes()[e] * model.materials()[e].lambda;
                volume += model.rest_volumes()[e];
            }
        }
        let lambda = model.node_lambdas()[node];
        assert!((lambda - weighted / volume).abs() <= 1e-12 * lambda);
        let pure = (lambda - soft).abs() <= 1e-9 * soft || (lambda - stiff).abs() <= 1e-9 * stiff;
        if !pure {
            assert!(lambda > soft && lambda < stiff);
            blended += 1;
        }
    }
    // The interface plane x = 0.01 holds 3 × 3 nodes.
    assert_eq!(blended, 9);
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

/// The f32 pipeline's forces, with every input rounded to f32 once.
fn f32_forces(model: &sim_soft_explicit::ExplicitModel, positions: &[[f64; 3]]) -> Vec<[f32; 3]> {
    use sim_soft_explicit::f32 as single;
    let narrow = |v: f64| v as f32;
    let positions: Vec<[f32; 3]> = positions.iter().map(|p| p.map(narrow)).collect();
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
    let pressures: Vec<f32> = current
        .iter()
        .zip(model.node_rest_volumes())
        .zip(model.node_lambdas())
        .map(|((&v, &rest), &lambda)| {
            single::pressure_lambda_term(
                single::nodal_volume_ratio(v, narrow(rest)),
                narrow(lambda),
            )
        })
        .collect();
    let mut forces = vec![[0.0_f32; 3]; model.node_count()];
    for (i, &e) in model.elements().iter().enumerate() {
        let m = model.materials()[i];
        let material = single::Material {
            mu: narrow(m.mu),
            lambda: narrow(m.lambda),
            c2: narrow(m.c2),
            density: narrow(m.density),
        };
        let pressure = single::element_pressure(e.map(|n| pressures[n as usize]));
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
    forces
}

#[test]
fn f32_forces_agree_with_f64_at_large_and_small_strain() {
    // The f32 error is roughly the same force whatever the strain, so at
    // small strain it is a larger share of the force. Judged against the
    // force a unit strain would make, `(λ + 2μ) h²` on a cell of side h.
    // Whether f32 is enough for the product's readings is plan §15a K3,
    // measured in build step 2.
    let side = 0.01;
    let model = block_model((2, 2, 2), side, YEOH);
    let unit_strain_force = (YEOH.lambda + 2.0 * YEOH.mu) * side * side;
    for amount in [0.2, 1e-3] {
        let deformed = deform(model.rest_positions(), amount);
        let reference = elastic_forces(&model, &deformed);
        let largest = max_abs(reference.iter().flatten().copied());
        let worst = max_abs(
            f32_forces(&model, &deformed)
                .iter()
                .zip(&reference)
                .flat_map(|(a, b)| (0..3).map(move |d| f64::from(a[d]) - b[d])),
        );
        eprintln!(
            "MARGIN f32 forces at deformation {amount}: {worst:e}, largest force {largest:e}, \
             unit-strain force {unit_strain_force:e}"
        );
        assert!(worst <= 1e-6 * unit_strain_force, "at {amount}: {worst:e}");
    }
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
