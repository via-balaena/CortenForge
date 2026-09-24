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
        let f = shared::tet4_deformation_gradient(x, model.rest_edge_inverses()[e]);
        let identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        assert!(
            (0..9).all(|i| (f[i] - identity[i]).abs() < 1e-12),
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
