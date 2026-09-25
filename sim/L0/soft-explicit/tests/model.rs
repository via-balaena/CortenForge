//! The lowered model: its rest-state quantities and its validation.

#![allow(clippy::unwrap_used, clippy::float_cmp)]

mod common;

use common::{SILICONE, block, block_model};
use sim_soft_explicit::f64 as shared;
use sim_soft_explicit::{ExplicitModel, ModelError};

fn unit_tet() -> (Vec<[f64; 3]>, Vec<[u32; 4]>) {
    (
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        vec![[0, 1, 2, 3]],
    )
}

fn build(positions: Vec<[f64; 3]>, elements: Vec<[u32; 4]>) -> Result<ExplicitModel, ModelError> {
    let (count, nodes) = (elements.len(), positions.len());
    ExplicitModel::new(
        positions,
        elements,
        vec![SILICONE; count],
        vec![false; nodes],
    )
}

#[test]
fn mass_and_volume_are_lumped_a_quarter_per_node() {
    let (n, side) = ((3, 2, 2), 0.01);
    let model = block_model(n, side, SILICONE);
    let volume = 12.0 * side * side * side;
    let total_volume: f64 = model.rest_volumes().iter().sum();
    assert!((total_volume - volume).abs() < 1e-15);
    let total_mass: f64 = model.node_masses().iter().sum();
    assert!((total_mass - SILICONE.density * volume).abs() < 1e-12);
    let nodal_volume: f64 = model.node_rest_volumes().iter().sum();
    assert!((nodal_volume - volume).abs() < 1e-15);
    assert_eq!(model.element_count(), 6 * 12);
    assert_eq!(model.node_count(), 4 * 3 * 3);
    assert_eq!(model.materials().len(), model.element_count());
    assert_eq!(model.held().len(), model.node_count());
}

#[test]
fn the_rest_edge_inverse_gives_the_identity_at_rest() {
    let model = block_model((2, 2, 1), 0.01, SILICONE);
    for (e, element) in model.elements().iter().enumerate() {
        let x = common::gather(model.rest_positions(), *element);
        let product = shared::mat3_mul(shared::tet4_edge_matrix(x), model.rest_edge_inverses()[e]);
        let identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        assert!(
            (0..9).all(|i| (product[i] - identity[i]).abs() < 1e-12),
            "element {e}"
        );
    }
}

#[test]
fn the_surface_is_the_boundary_wound_outward() {
    // A conforming n × n × n block has 2 triangles per boundary square.
    let n: u32 = 3;
    let side = 0.01;
    let cells = n as usize;
    let model = block_model((cells, cells, cells), side, SILICONE);
    assert_eq!(model.surface_triangles().len(), 12 * cells * cells);
    let center = [0.5 * f64::from(n) * side; 3];
    for triangle in model.surface_triangles() {
        let [a, b, c] = triangle.map(|i| model.rest_positions()[i as usize]);
        let normal = shared::vec3_cross(shared::vec3_sub(b, a), shared::vec3_sub(c, a));
        let centroid = [0, 1, 2].map(|d| (a[d] + b[d] + c[d]) / 3.0);
        assert!(shared::vec3_dot(normal, shared::vec3_sub(centroid, center)) > 0.0);
    }
}

#[test]
fn each_problem_is_named() {
    let (positions, elements) = unit_tet();
    assert_eq!(
        build(positions.clone(), vec![]).unwrap_err(),
        ModelError::Empty
    );

    let err = ExplicitModel::new(positions.clone(), elements.clone(), vec![], vec![false; 4])
        .unwrap_err();
    assert_eq!(
        err,
        ModelError::LengthMismatch {
            what: "materials",
            found: 0,
            expected: 1
        }
    );
    let err = ExplicitModel::new(
        positions.clone(),
        elements.clone(),
        vec![SILICONE],
        vec![false; 3],
    )
    .unwrap_err();
    assert!(matches!(
        err,
        ModelError::LengthMismatch { what: "held", .. }
    ));

    let mut bad = positions.clone();
    bad[2][1] = f64::NAN;
    assert_eq!(
        build(bad, elements.clone()).unwrap_err(),
        ModelError::NonFinitePosition { node: 2 }
    );

    let err = build(positions.clone(), vec![[0, 1, 2, 7]]).unwrap_err();
    assert_eq!(
        err,
        ModelError::NodeOutOfRange {
            element: 0,
            node: 7,
            nodes: 4
        }
    );
    let err = build(positions.clone(), vec![[0, 1, 1, 3]]).unwrap_err();
    assert_eq!(
        err,
        ModelError::RepeatedNode {
            element: 0,
            node: 1
        }
    );

    let err = build(positions.clone(), vec![[0, 2, 1, 3]]).unwrap_err();
    assert!(matches!(err, ModelError::NonPositiveVolume { element: 0, volume } if volume < 0.0));
    assert!(err.to_string().contains("wrong way round"));

    let mut extra = positions;
    extra.push([5.0, 5.0, 5.0]);
    assert_eq!(
        build(extra, elements).unwrap_err(),
        ModelError::UnreferencedNode { node: 4 }
    );
}

#[test]
fn an_out_of_range_material_is_rejected() {
    let (positions, elements) = unit_tet();
    let cases = [
        (
            shared::Material {
                mu: 0.0,
                ..SILICONE
            },
            "shear modulus",
        ),
        (
            shared::Material {
                lambda: -SILICONE.mu,
                ..SILICONE
            },
            "bulk modulus",
        ),
        (
            shared::Material {
                c2: -1.0,
                ..SILICONE
            },
            "C₂",
        ),
        (
            shared::Material {
                density: 0.0,
                ..SILICONE
            },
            "density",
        ),
        (
            shared::Material {
                lambda: f64::INFINITY,
                ..SILICONE
            },
            "not finite",
        ),
    ];
    for (material, reason) in cases {
        let err = ExplicitModel::new(
            positions.clone(),
            elements.clone(),
            vec![material],
            vec![false; 4],
        )
        .unwrap_err();
        assert!(
            matches!(&err, ModelError::InvalidMaterial { element: 0, reason: r } if r.contains(reason)),
            "{err:?}"
        );
    }
}

#[test]
fn held_nodes_and_per_element_materials_are_kept() {
    let (positions, elements) = block((1, 1, 1), 1.0);
    let stiff = shared::Material {
        mu: 2.0 * SILICONE.mu,
        ..SILICONE
    };
    let materials: Vec<_> = (0..elements.len())
        .map(|e| if e % 2 == 0 { stiff } else { SILICONE })
        .collect();
    let held: Vec<bool> = (0..positions.len()).map(|n| n < 4).collect();
    let model = ExplicitModel::new(positions, elements, materials.clone(), held.clone()).unwrap();
    assert_eq!(model.materials(), materials.as_slice());
    assert_eq!(model.held(), held.as_slice());
}

#[test]
fn each_node_lists_the_slots_that_refer_to_it_in_order() {
    let model = block_model((2, 1, 1), 1.0, SILICONE);
    let incidence = model.element_incidence();
    assert_eq!(incidence.offsets().len(), model.node_count() + 1);
    assert_eq!(incidence.entries().len(), 4 * model.element_count());
    for node in 0..model.node_count() {
        let slots = incidence.of(node);
        assert!(
            slots.windows(2).all(|w| w[0] < w[1]),
            "node {node}: {slots:?}"
        );
        for &slot in slots {
            let (element, corner) = (slot as usize / 4, slot as usize % 4);
            assert_eq!(model.elements()[element][corner] as usize, node);
        }
    }
    // Every slot appears exactly once.
    let mut all: Vec<u32> = incidence.entries().to_vec();
    all.sort_unstable();
    assert!(all.iter().enumerate().all(|(i, &s)| s as usize == i));
    let surface = model.surface_incidence();
    for node in 0..model.node_count() {
        for &slot in surface.of(node) {
            let (triangle, corner) = (slot as usize / 3, slot as usize % 3);
            assert_eq!(model.surface_triangles()[triangle][corner] as usize, node);
        }
    }
    // A 2 × 1 × 1 block has every node on its surface.
    assert!((0..model.node_count()).all(|n| !surface.of(n).is_empty()));
}

#[test]
fn constraints_must_be_unit_or_zero_and_orthogonal() {
    let model = block_model((1, 1, 1), 1.0, SILICONE);
    let nodes = model.node_count();
    let free = [[0.0; 3]; 2];
    assert!(model.constraints().iter().all(|c| *c == free));
    let radial_axial = [[0.6, 0.8, 0.0], [0.0, 0.0, 1.0]];
    let mut constraints = vec![free; nodes];
    constraints[3] = radial_axial;
    let constrained = model.clone().with_constraints(constraints.clone()).unwrap();
    assert_eq!(constrained.constraints()[3], radial_axial);
    for (bad, reason) in [
        ([[0.5, 0.0, 0.0], [0.0; 3]], "neither zero nor unit"),
        ([[1.0, 0.0, 0.0], [0.6, 0.8, 0.0]], "not orthogonal"),
        ([[f64::NAN, 0.0, 0.0], [0.0; 3]], "not finite"),
    ] {
        constraints[5] = bad;
        let err = model
            .clone()
            .with_constraints(constraints.clone())
            .unwrap_err();
        assert!(
            matches!(&err, ModelError::InvalidConstraint { node: 5, reason: r } if r.contains(reason)),
            "{err:?}"
        );
    }
    let err = model.with_constraints(vec![free; nodes - 1]).unwrap_err();
    assert!(matches!(
        err,
        ModelError::LengthMismatch {
            what: "constraints",
            ..
        }
    ));
}
