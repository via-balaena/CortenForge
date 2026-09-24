//! Small meshes and the executor-side steps (gathers) the tests need.

#![allow(
    dead_code,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss
)]

use sim_soft_explicit::f64 as shared;
use sim_soft_explicit::{ExplicitModel, f64::Material};

/// Silicone-like: μ 23 kPa, ν 0.49, ρ 1070 kg/m³ (plan §15b's material).
pub const SILICONE: Material = Material {
    mu: 23.0e3,
    lambda: 23.0e3 * 2.0 * 0.49 / (1.0 - 2.0 * 0.49),
    c2: 0.0,
    density: 1070.0,
};

/// A block of `n.0 × n.1 × n.2` cubes of side `side`, each split into six
/// tetrahedra around the same body diagonal (Kuhn), so neighbouring cubes'
/// faces match. Returns node positions and positively oriented elements.
pub fn block(n: (usize, usize, usize), side: f64) -> (Vec<[f64; 3]>, Vec<[u32; 4]>) {
    let node = |i: usize, j: usize, k: usize| ((k * (n.1 + 1) + j) * (n.0 + 1) + i) as u32;
    let mut positions = Vec::new();
    for k in 0..=n.2 {
        for j in 0..=n.1 {
            for i in 0..=n.0 {
                positions.push([i as f64 * side, j as f64 * side, k as f64 * side]);
            }
        }
    }
    let mut elements = Vec::new();
    let axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    let orders = [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ];
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
                    if shared::tet4_volume(gather(&positions, tet)) < 0.0 {
                        tet.swap(1, 2);
                    }
                    elements.push(tet);
                }
            }
        }
    }
    (positions, elements)
}

/// One element's node positions, as the shared math takes them.
pub fn gather(positions: &[[f64; 3]], element: [u32; 4]) -> [f64; 12] {
    let mut x = [0.0; 12];
    for (slot, &node) in element.iter().enumerate() {
        x[3 * slot..3 * slot + 3].copy_from_slice(&positions[node as usize]);
    }
    x
}

/// A smooth, non-uniform deformation of `positions` (stretch, shear, twist
/// and a ripple), keeping every element positively oriented for the small
/// meshes here.
pub fn deform(positions: &[[f64; 3]], amount: f64) -> Vec<[f64; 3]> {
    positions
        .iter()
        .map(|&[x, y, z]| {
            [
                x + amount * (0.3 * x + 0.2 * y * z + 0.05 * (7.0 * y).sin()),
                y + amount * (-0.25 * y + 0.15 * x + 0.04 * (5.0 * z).cos()),
                z + amount * (0.1 * z - 0.2 * x * y + 0.06 * (6.0 * x).sin()),
            ]
        })
        .collect()
}

/// The forces of the whole elastic pipeline (plan §15f phases 1–5) at
/// `positions`, at f64.
pub fn elastic_forces(model: &ExplicitModel, positions: &[[f64; 3]]) -> Vec<[f64; 3]> {
    let nodal_ratio = nodal_volume_ratios(model, positions);
    let per_lambda: Vec<f64> = nodal_ratio
        .iter()
        .map(|&j| shared::pressure_per_lambda(j))
        .collect();
    let mut forces = vec![[0.0; 3]; model.node_count()];
    for (e, element) in model.elements().iter().enumerate() {
        let material = model.materials()[e];
        let pressure =
            shared::element_pressure(material.lambda, element.map(|n| per_lambda[n as usize]));
        let f = shared::tet4_elastic_forces(
            gather(positions, *element),
            model.rest_edge_inverses()[e],
            model.rest_volumes()[e],
            material,
            pressure,
        );
        for (slot, &node) in element.iter().enumerate() {
            for d in 0..3 {
                forces[node as usize][d] += f[3 * slot + d];
            }
        }
    }
    forces
}

/// Each node's volume ratio `J_a = v_a / V_a` (phases 1–3).
pub fn nodal_volume_ratios(model: &ExplicitModel, positions: &[[f64; 3]]) -> Vec<f64> {
    let mut current = vec![0.0; model.node_count()];
    for element in model.elements() {
        let v = shared::tet4_volume(gather(positions, *element));
        for &node in element {
            current[node as usize] += 0.25 * v;
        }
    }
    current
        .iter()
        .zip(model.node_rest_volumes())
        .map(|(&v, &rest)| shared::nodal_volume_ratio(v, rest))
        .collect()
}

/// The energy whose gradient the pipeline's forces are, inside one material:
/// `Σ_e V_e Ψ_μ(F_e) + Σ_a V_a λ/2 (ln J_a)²`.
pub fn elastic_energy(model: &ExplicitModel, positions: &[[f64; 3]]) -> f64 {
    let mu_terms: f64 = model
        .elements()
        .iter()
        .enumerate()
        .map(|(e, element)| {
            shared::tet4_energy_mu_terms(
                gather(positions, *element),
                model.rest_edge_inverses()[e],
                model.rest_volumes()[e],
                model.materials()[e],
            )
        })
        .sum();
    let lambda = model.materials()[0].lambda;
    let lambda_term: f64 = nodal_volume_ratios(model, positions)
        .iter()
        .zip(model.node_rest_volumes())
        .map(|(&j, &rest)| rest * shared::energy_density_lambda_term(j, lambda))
        .sum();
    mu_terms + lambda_term
}

/// A model of `block(n, side)` in one material, nothing held.
pub fn block_model(n: (usize, usize, usize), side: f64, material: Material) -> ExplicitModel {
    let (positions, elements) = block(n, side);
    let count = elements.len();
    let nodes = positions.len();
    ExplicitModel::new(
        positions,
        elements,
        vec![material; count],
        vec![false; nodes],
    )
    .unwrap_or_else(|e| unreachable!("the block mesh is valid: {e}"))
}
