//! The lowered model: the flat arrays every executor uploads, validated, with
//! the rest-state quantities computed once in `f64`.

use std::collections::BTreeMap;

use crate::f64::{
    Material, mat3_cofactor, mat3_scale, mat3_transpose, tet4_edge_matrix, tet4_volume,
};

/// A lowered four-node-tetrahedron model, ready for an explicit executor.
///
/// Built by [`ExplicitModel::new`], which validates the input and computes
/// each element's rest volume and inverse rest edge matrix, each node's
/// lumped mass, tributary rest volume and λ, and the boundary surface.
#[derive(Clone, Debug)]
pub struct ExplicitModel {
    rest_positions: Vec<[f64; 3]>,
    elements: Vec<[u32; 4]>,
    materials: Vec<Material>,
    held: Vec<bool>,
    rest_edge_inverses: Vec<[f64; 9]>,
    rest_volumes: Vec<f64>,
    node_masses: Vec<f64>,
    node_rest_volumes: Vec<f64>,
    node_lambdas: Vec<f64>,
    surface: Vec<[u32; 3]>,
}

/// Why a model was rejected.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum ModelError {
    /// The model has no elements.
    #[error("the model has no elements")]
    Empty,
    /// A per-element or per-node array has the wrong length.
    #[error("{what} has {found} entries; expected {expected}")]
    LengthMismatch {
        /// Which array.
        what: &'static str,
        /// Its length.
        found: usize,
        /// The length it needs.
        expected: usize,
    },
    /// A node position is not finite.
    #[error("node {node}'s position is not finite")]
    NonFinitePosition {
        /// The node.
        node: usize,
    },
    /// An element refers to a node that does not exist.
    #[error("element {element} refers to node {node}, but there are {nodes} nodes")]
    NodeOutOfRange {
        /// The element.
        element: usize,
        /// The missing node.
        node: u32,
        /// How many nodes there are.
        nodes: usize,
    },
    /// An element lists one node twice.
    #[error("element {element} lists node {node} twice")]
    RepeatedNode {
        /// The element.
        element: usize,
        /// The repeated node.
        node: u32,
    },
    /// An element's rest volume is not positive: it is flat, or its nodes are
    /// ordered the wrong way round.
    #[error(
        "element {element} has rest volume {volume:e}; it must be positive (flat, or nodes ordered the wrong way round)"
    )]
    NonPositiveVolume {
        /// The element.
        element: usize,
        /// Its signed rest volume.
        volume: f64,
    },
    /// An element's material parameters are out of range.
    #[error("element {element}'s material is invalid: {reason}")]
    InvalidMaterial {
        /// The element.
        element: usize,
        /// What is wrong.
        reason: &'static str,
    },
    /// A node belongs to no element, so it would have no mass.
    #[error("node {node} belongs to no element, so it has no mass")]
    UnreferencedNode {
        /// The node.
        node: usize,
    },
}

impl ExplicitModel {
    /// Validate a model and compute its rest state.
    ///
    /// - `rest_positions`: one per node.
    /// - `elements`: four node indices each, ordered so the rest volume is
    ///   positive ([`crate::f64::tet4_volume`]).
    /// - `materials`: one per element.
    /// - `held`: one per node; a held node stays at its rest position.
    ///
    /// # Errors
    /// A [`ModelError`] naming the first problem found.
    pub fn new(
        rest_positions: Vec<[f64; 3]>,
        elements: Vec<[u32; 4]>,
        materials: Vec<Material>,
        held: Vec<bool>,
    ) -> Result<Self, ModelError> {
        if elements.is_empty() {
            return Err(ModelError::Empty);
        }
        check_length("materials", materials.len(), elements.len())?;
        check_length("held", held.len(), rest_positions.len())?;
        if let Some(node) = rest_positions
            .iter()
            .position(|p| !p.iter().all(|c| c.is_finite()))
        {
            return Err(ModelError::NonFinitePosition { node });
        }

        let nodes = rest_positions.len();
        let mut rest_edge_inverses = Vec::with_capacity(elements.len());
        let mut rest_volumes = Vec::with_capacity(elements.len());
        let mut node_masses = vec![0.0; nodes];
        let mut node_rest_volumes = vec![0.0; nodes];
        let mut node_lambda_volumes = vec![0.0; nodes];
        for (element, (corners, material)) in elements.iter().zip(&materials).enumerate() {
            let x = gather(&rest_positions, *corners, element)?;
            check_material(element, material)?;
            let volume = tet4_volume(x);
            if volume <= 0.0 {
                return Err(ModelError::NonPositiveVolume { element, volume });
            }
            let edges = tet4_edge_matrix(x);
            rest_edge_inverses.push(mat3_scale(
                mat3_transpose(mat3_cofactor(edges)),
                1.0 / (6.0 * volume),
            ));
            rest_volumes.push(volume);
            for &node in corners {
                node_masses[node as usize] += 0.25 * material.density * volume;
                node_rest_volumes[node as usize] += 0.25 * volume;
                node_lambda_volumes[node as usize] += 0.25 * volume * material.lambda;
            }
        }
        if let Some(node) = node_rest_volumes.iter().position(|&v| v == 0.0) {
            return Err(ModelError::UnreferencedNode { node });
        }
        let node_lambdas = node_lambda_volumes
            .iter()
            .zip(&node_rest_volumes)
            .map(|(weighted, volume)| weighted / volume)
            .collect();
        let surface = boundary_faces(&elements);

        Ok(Self {
            rest_positions,
            elements,
            materials,
            held,
            rest_edge_inverses,
            rest_volumes,
            node_masses,
            node_rest_volumes,
            node_lambdas,
            surface,
        })
    }

    /// The number of nodes.
    #[must_use]
    pub const fn node_count(&self) -> usize {
        self.rest_positions.len()
    }

    /// The number of elements.
    #[must_use]
    pub const fn element_count(&self) -> usize {
        self.elements.len()
    }

    /// Node rest positions.
    #[must_use]
    pub fn rest_positions(&self) -> &[[f64; 3]] {
        &self.rest_positions
    }

    /// Element node indices.
    #[must_use]
    pub fn elements(&self) -> &[[u32; 4]] {
        &self.elements
    }

    /// Each element's material.
    #[must_use]
    pub fn materials(&self) -> &[Material] {
        &self.materials
    }

    /// Whether each node is held at its rest position.
    #[must_use]
    pub fn held(&self) -> &[bool] {
        &self.held
    }

    /// Each element's inverse rest edge matrix, row-major
    /// ([`crate::f64::tet4_edge_matrix`]).
    #[must_use]
    pub fn rest_edge_inverses(&self) -> &[[f64; 9]] {
        &self.rest_edge_inverses
    }

    /// Each element's rest volume.
    #[must_use]
    pub fn rest_volumes(&self) -> &[f64] {
        &self.rest_volumes
    }

    /// Each node's lumped mass: a quarter of each incident element's mass
    /// (the implicit solver's rule).
    #[must_use]
    pub fn node_masses(&self) -> &[f64] {
        &self.node_masses
    }

    /// Each node's tributary rest volume, a quarter of each incident
    /// element's: the `V_a` of averaged nodal pressure.
    #[must_use]
    pub fn node_rest_volumes(&self) -> &[f64] {
        &self.node_rest_volumes
    }

    /// Each node's λ for averaged nodal pressure: the λ of the elements around
    /// it, weighted by rest volume.
    ///
    /// Inside one material it is that material's λ, so averaged nodal
    /// pressure is plain selective ANP there. Where materials meet it blends
    /// them, and the forces stay the exact gradient of an energy
    /// (`Σ_a V_a λ_a/2 (ln J_a)²` for the λ term). This interface rule is
    /// provisional: plan §15g step 1 records the alternative and what would
    /// decide between them.
    #[must_use]
    pub fn node_lambdas(&self) -> &[f64] {
        &self.node_lambdas
    }

    /// The boundary triangles (faces that belong to one element), each wound
    /// so its normal points out of the body, in a deterministic order.
    #[must_use]
    pub fn surface_triangles(&self) -> &[[u32; 3]] {
        &self.surface
    }
}

const fn check_length(what: &'static str, found: usize, expected: usize) -> Result<(), ModelError> {
    if found == expected {
        Ok(())
    } else {
        Err(ModelError::LengthMismatch {
            what,
            found,
            expected,
        })
    }
}

/// One element's node positions, as the shared math takes them.
fn gather(
    positions: &[[f64; 3]],
    corners: [u32; 4],
    element: usize,
) -> Result<[f64; 12], ModelError> {
    let mut x = [0.0; 12];
    for (slot, &node) in corners.iter().enumerate() {
        if corners[..slot].contains(&node) {
            return Err(ModelError::RepeatedNode { element, node });
        }
        let p = positions
            .get(node as usize)
            .ok_or(ModelError::NodeOutOfRange {
                element,
                node,
                nodes: positions.len(),
            })?;
        x[3 * slot..3 * slot + 3].copy_from_slice(p);
    }
    Ok(x)
}

fn check_material(element: usize, m: &Material) -> Result<(), ModelError> {
    let reason = if ![m.mu, m.lambda, m.c2, m.density]
        .iter()
        .all(|v| v.is_finite())
    {
        Some("a parameter is not finite")
    } else if m.mu <= 0.0 {
        Some("the shear modulus must be positive")
    } else if m.lambda + 2.0 / 3.0 * m.mu <= 0.0 {
        Some("the bulk modulus, λ + 2μ/3, must be positive")
    } else if m.c2 < 0.0 {
        Some("Yeoh's C₂ must not be negative")
    } else if m.density <= 0.0 {
        Some("the density must be positive")
    } else {
        None
    };
    reason.map_or(Ok(()), |reason| {
        Err(ModelError::InvalidMaterial { element, reason })
    })
}

/// The faces that belong to exactly one element, wound outward.
///
/// For a positively oriented element `(a, b, c, d)` the outward faces are
/// `(a, c, b)`, `(a, b, d)`, `(a, d, c)` and `(b, c, d)`.
fn boundary_faces(elements: &[[u32; 4]]) -> Vec<[u32; 3]> {
    let mut faces: BTreeMap<[u32; 3], (usize, [u32; 3])> = BTreeMap::new();
    for &[a, b, c, d] in elements {
        for face in [[a, c, b], [a, b, d], [a, d, c], [b, c, d]] {
            let mut key = face;
            key.sort_unstable();
            faces.entry(key).or_insert((0, face)).0 += 1;
        }
    }
    faces
        .into_values()
        .filter(|&(count, _)| count == 1)
        .map(|(_, face)| face)
        .collect()
}
