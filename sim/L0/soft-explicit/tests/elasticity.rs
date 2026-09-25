//! The constitutive law, the element and averaged nodal pressure, checked
//! against definitions: stress is the energy's derivative, forces are the
//! mesh energy's gradient, both are frame indifferent, and the expansions
//! in the displacement gradient agree with the direct formulas.

#![allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
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

/// The displacement gradient `H = F − I` of a deformation gradient.
const fn h_of(f: [f64; 9]) -> [f64; 9] {
    shared::mat3_sub(f, IDENTITY)
}

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

/// `P(F)` straight from `F`, as `sim-soft` writes it:
/// `μ (F − F⁻ᵀ) + 4 C₂ (I₁ − 3) F + λ ln J F⁻ᵀ`.
fn direct_first_piola(f: [f64; 9], material: Material) -> [f64; 9] {
    let j = shared::mat3_det(f);
    let f_inverse_transpose = shared::mat3_scale(shared::mat3_cofactor(f), 1.0 / j);
    let i1_minus_3 = shared::mat3_frobenius_squared(f) - 3.0;
    shared::mat3_add(
        shared::mat3_add(
            shared::mat3_scale(shared::mat3_sub(f, f_inverse_transpose), material.mu),
            shared::mat3_scale(f, 4.0 * material.c2 * i1_minus_3),
        ),
        shared::mat3_scale(f_inverse_transpose, material.lambda * j.ln()),
    )
}

#[test]
fn the_rest_state_is_stress_and_energy_free() {
    for material in [SILICONE, YEOH] {
        assert_eq!(shared::first_piola([0.0; 9], material), [0.0; 9]);
        assert_eq!(shared::energy_density([0.0; 9], material), 0.0);
    }
}

#[test]
fn the_expansions_agree_with_the_direct_formulas() {
    // At a large strain, where the direct formulas lose nothing, the
    // expansions in H must give the same numbers.
    let h = h_of(F);
    let dilation = shared::gradient_dilation(h);
    assert!((dilation - (shared::mat3_det(F) - 1.0)).abs() < 1e-15);
    let cofactor = shared::deformation_cofactor(h);
    let direct = shared::mat3_cofactor(F);
    assert!(max_abs((0..9).map(|i| cofactor[i] - direct[i])) < 1e-15);
    for material in [SILICONE, YEOH] {
        let p = shared::first_piola(h, material);
        let direct = direct_first_piola(F, material);
        assert!(max_abs((0..9).map(|i| p[i] - direct[i])) <= 1e-13 * max_abs(direct));
    }
}

#[test]
fn ln_1p_matches_the_standard_library() {
    use sim_soft_explicit::f32 as single;
    let (mut worst64, mut worst32) = (0.0_f64, 0.0_f64);
    for i in -900..=3000 {
        for x in [f64::from(i) * 1e-3, f64::from(i) * 1e-7] {
            let exact = x.ln_1p();
            if exact == 0.0 {
                assert_eq!(shared::ln_1p(x), 0.0);
                continue;
            }
            worst64 = worst64.max(((shared::ln_1p(x) - exact) / exact).abs());
            let x32 = x as f32;
            let exact32 = f64::from(x32).ln_1p();
            worst32 = worst32.max(((f64::from(single::ln_1p(x32)) - exact32) / exact32).abs());
        }
    }
    eprintln!("MARGIN ln_1p relative error: f64 {worst64:e}, f32 {worst32:e}");
    assert!(worst64 <= 4.0 * f64::EPSILON, "f64 {worst64:e}");
    assert!(worst32 <= 4.0 * f64::from(f32::EPSILON), "f32 {worst32:e}");
}

#[test]
fn the_stress_is_the_energys_derivative() {
    for material in [SILICONE, YEOH] {
        let h = h_of(F);
        let p = shared::first_piola(h, material);
        for i in 0..9 {
            let step = 1e-6;
            let mut plus = h;
            let mut minus = h;
            plus[i] += step;
            minus[i] -= step;
            let derivative = (shared::energy_density(plus, material)
                - shared::energy_density(minus, material))
                / (2.0 * step);
            assert!(
                (derivative - p[i]).abs() <= 1e-6 * max_abs(p),
                "entry {i}: dΨ/dH {derivative} against P {}",
                p[i]
            );
        }
    }
}

#[test]
fn the_split_adds_up_to_the_full_law() {
    for material in [SILICONE, YEOH] {
        let h = h_of(F);
        let d = shared::gradient_dilation(h);
        let lambda_term = shared::mat3_scale(
            shared::deformation_cofactor(h),
            shared::pressure_lambda_term(d, material.lambda),
        );
        let split = shared::mat3_add(shared::first_piola_mu_terms(h, material), lambda_term);
        let full = shared::first_piola(h, material);
        assert!(max_abs((0..9).map(|i| split[i] - full[i])) <= 1e-12 * max_abs(full));
        let energy = shared::energy_density_mu_terms(h, material)
            + shared::energy_density_lambda_term(d, material.lambda);
        assert_eq!(energy, shared::energy_density(h, material));
        // U′(J) = λ ln J / J, per unit λ.
        let step = 1e-7;
        let derivative = (shared::energy_density_lambda_term(d + step, 1.0)
            - shared::energy_density_lambda_term(d - step, 1.0))
            / (2.0 * step);
        assert!((derivative - shared::pressure_lambda_term(d, 1.0)).abs() < 1e-8);
    }
}

#[test]
fn uniaxial_stretch_matches_the_closed_form() {
    // F = diag(s, 1, 1): P₁₁ = μ(s − 1/s) + λ ln s / s + 4 C₂ (s² − 1) s.
    let s: f64 = 1.3;
    let h = [s - 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    for material in [SILICONE, YEOH] {
        let expected = material.mu * (s - 1.0 / s)
            + material.lambda * s.ln() / s
            + 4.0 * material.c2 * (s * s - 1.0) * s;
        let p = shared::first_piola(h, material);
        assert!((p[0] - expected).abs() <= 1e-12 * expected.abs());
        assert!(max_abs([p[1], p[2], p[3], p[5], p[6], p[7]]) < 1e-9);
    }
}

#[test]
fn the_stress_is_frame_indifferent() {
    // P(QF) = Q P(F) for a rotation Q.
    let q = rotation(0.7);
    for material in [SILICONE, YEOH] {
        let rotated = shared::first_piola(h_of(shared::mat3_mul(q, F)), material);
        let expected = shared::mat3_mul(q, shared::first_piola(h_of(F), material));
        assert!(max_abs((0..9).map(|i| rotated[i] - expected[i])) <= 1e-10 * max_abs(expected));
    }
}

#[test]
fn inversion_is_reported_and_the_stress_stays_finite() {
    let inverted = h_of([-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
    assert!(shared::is_inverted(shared::gradient_dilation(inverted)));
    assert!(shared::is_inverted(-1.0));
    assert!(!shared::is_inverted(-1.0 + 1e-12));
    assert!(
        shared::first_piola(inverted, YEOH)
            .iter()
            .all(|v| v.is_finite())
    );
    assert!(shared::energy_density(inverted, YEOH).is_finite());
    assert!(shared::pressure_lambda_term(-1.5, YEOH.lambda).is_finite());
}

#[test]
fn one_tets_forces_are_its_energys_gradient_and_balance() {
    let rest = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    assert_eq!(shared::tet4_volume(rest), 1.0 / 6.0);
    // For this tet the rest edge matrix is I, so its inverse is too.
    let rest_inverse = IDENTITY;
    let u = [
        0.05, -0.02, 0.01, 0.2, 0.1, -0.05, -0.1, -0.1, 0.08, 0.03, 0.12, 0.1,
    ];
    let energy = |u: [f64; 12]| shared::tet4_energy_mu_terms(u, rest_inverse, 1.0 / 6.0, YEOH);
    let forces = shared::tet4_elastic_forces(u, rest_inverse, 1.0 / 6.0, YEOH, 0.0);
    for i in 0..12 {
        let step = 1e-7;
        let mut plus = u;
        let mut minus = u;
        plus[i] += step;
        minus[i] -= step;
        let gradient = (energy(plus) - energy(minus)) / (2.0 * step);
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

/// The f32 pipeline's nodal pressures and forces from f64 positions: the
/// displacements, rest data and materials are each rounded to f32 once, as
/// an executor that stores displacements would hold them.
fn f32_pipeline(
    model: &sim_soft_explicit::ExplicitModel,
    positions: &[[f64; 3]],
) -> (Vec<f32>, Vec<[f32; 3]>) {
    use sim_soft_explicit::f32 as single;
    let narrow = |v: f64| v as f32;
    let u: Vec<[f32; 3]> = common::displacements(model, positions)
        .iter()
        .map(|d| d.map(narrow))
        .collect();
    let gather = |e: [u32; 4]| {
        let mut x = [0.0_f32; 12];
        for (slot, &n) in e.iter().enumerate() {
            x[3 * slot..3 * slot + 3].copy_from_slice(&u[n as usize]);
        }
        x
    };
    let mut change = vec![0.0_f32; model.node_count()];
    for (i, &e) in model.elements().iter().enumerate() {
        let d = single::tet4_dilation(gather(e), model.rest_edge_inverses()[i].map(narrow));
        for n in e {
            change[n as usize] += 0.25 * narrow(model.rest_volumes()[i]) * d;
        }
    }
    let pressures: Vec<f32> = change
        .iter()
        .zip(model.node_rest_volumes())
        .zip(model.node_lambdas())
        .map(|((&dv, &rest), &lambda)| {
            single::pressure_lambda_term(single::nodal_dilation(dv, narrow(rest)), narrow(lambda))
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
    (pressures, forces)
}

#[test]
fn f32_forces_agree_with_f64_at_large_and_small_strain() {
    // Judged against the force a unit strain would make, `(λ + 2μ) h²` on a
    // cell of side h, since at small strain the forces themselves are small.
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
            f32_pipeline(&model, &deformed)
                .1
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
fn f32_nodal_pressure_holds_far_from_the_origin_at_high_nu() {
    // The case that f32 positions fail (plan §6): cells of 1.5 mm, 0.12 m
    // from the origin, turned 0.4 rad, compressed to the band's pressure,
    // 0.12 μ (plan §15b), with a small shear on top. Each node's f32
    // pressure against f64, as a share of 0.12 μ. This measures one
    // evaluation, not an accumulating run.
    let offset = [0.12, -0.07, 0.09];
    let q = rotation(0.4);
    for nu in [0.49, 0.495, 0.4995] {
        let material = Material {
            lambda: 2.0 * SILICONE.mu * nu / (1.0 - 2.0 * nu),
            ..SILICONE
        };
        let (cells, side) = ((4, 4, 4), 0.0015);
        let (local, elements) = common::block(cells, side);
        let rest: Vec<[f64; 3]> = local
            .iter()
            .map(|&p| {
                let r = [
                    q[0] * p[0] + q[1] * p[1] + q[2] * p[2],
                    q[3] * p[0] + q[4] * p[1] + q[5] * p[2],
                    q[6] * p[0] + q[7] * p[1] + q[8] * p[2],
                ];
                [r[0] + offset[0], r[1] + offset[1], r[2] + offset[2]]
            })
            .collect();
        let count = elements.len();
        let nodes = rest.len();
        let model = sim_soft_explicit::ExplicitModel::new(
            rest.clone(),
            elements,
            vec![material; count],
            vec![false; nodes],
        )
        .unwrap();
        // p = λ ln J / J = −0.12 μ, so J − 1 ≈ −0.12 μ / λ.
        let scale = (1.0 - 0.12 * material.mu / material.lambda).cbrt();
        let deformed: Vec<[f64; 3]> = rest
            .iter()
            .map(|&p| {
                let r = [p[0] - offset[0], p[1] - offset[1], p[2] - offset[2]];
                [
                    offset[0] + scale * r[0] + 0.01 * r[1],
                    offset[1] + scale * r[1],
                    offset[2] + scale * r[2],
                ]
            })
            .collect();
        let reference: Vec<f64> = common::nodal_dilations(&model, &deformed)
            .iter()
            .zip(model.node_lambdas())
            .map(|(&d, &lambda)| shared::pressure_lambda_term(d, lambda))
            .collect();
        let band = 0.12 * material.mu;
        assert!(
            reference
                .iter()
                .all(|p| (p.abs() - band).abs() < 0.2 * band)
        );
        let (pressures, _) = f32_pipeline(&model, &deformed);
        let worst = pressures
            .iter()
            .zip(&reference)
            .map(|(&a, &b)| (f64::from(a) - b).abs() / band)
            .fold(0.0, f64::max);
        eprintln!("MARGIN f32 nodal pressure at nu {nu}, 0.12 m out: {worst:e} of 0.12 mu");
        assert!(worst <= 1e-4, "nu {nu}: {worst:e}");
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
