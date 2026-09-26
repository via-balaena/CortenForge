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
/// lumped mass, tributary rest volume and λ, the boundary surface, and the
/// incidence lists the executors' gathers walk. Per-direction constraints
/// are added with [`ExplicitModel::with_constraints`].
#[derive(Clone, Debug)]
pub struct ExplicitModel {
    rest_positions: Vec<[f64; 3]>,
    elements: Vec<[u32; 4]>,
    materials: Vec<Material>,
    held: Vec<bool>,
    constraints: Vec<[[f64; 3]; 2]>,
    rest_edge_inverses: Vec<[f64; 9]>,
    rest_volumes: Vec<f64>,
    node_masses: Vec<f64>,
    node_rest_volumes: Vec<f64>,
    node_lambdas: Vec<f64>,
    surface: Vec<[u32; 3]>,
    element_incidence: Incidence,
    surface_incidence: Incidence,
}

/// For each node, the slots that refer to it: a compressed list, node `n`'s
/// entries at `entries[offsets[n]..offsets[n + 1]]`, in ascending order.
///
/// An executor's gather sums these slots for each node, in this order, so no
/// two threads add into one place and the sum does not depend on how the
/// work is split (plan §16e).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Incidence {
    offsets: Vec<u32>,
    entries: Vec<u32>,
}

impl Incidence {
    /// Build from `items` of `N` node indices each; an entry is
    /// `item · N + corner`.
    fn new<const N: usize>(items: &[[u32; N]], nodes: usize) -> Result<Self, ModelError> {
        u32::try_from(items.len().saturating_mul(N)).map_err(|_| ModelError::TooLarge)?;
        let mut counts = vec![0_u32; nodes + 1];
        for item in items {
            for &node in item {
                counts[node as usize + 1] += 1;
            }
        }
        for n in 0..nodes {
            counts[n + 1] += counts[n];
        }
        let offsets = counts;
        let mut next = offsets.clone();
        let mut entries = vec![0_u32; offsets[nodes] as usize];
        for (index, item) in items.iter().enumerate() {
            for (corner, &node) in item.iter().enumerate() {
                let slot = &mut next[node as usize];
                entries[*slot as usize] =
                    u32::try_from(index * N + corner).map_err(|_| ModelError::TooLarge)?;
                *slot += 1;
            }
        }
        Ok(Self { offsets, entries })
    }

    /// The offsets, one per node plus one.
    #[must_use]
    pub fn offsets(&self) -> &[u32] {
        &self.offsets
    }

    /// The entries, `item · N + corner`, grouped by node.
    #[must_use]
    pub fn entries(&self) -> &[u32] {
        &self.entries
    }

    /// The entries that refer to `node`.
    ///
    /// # Panics
    /// If `node` is out of range.
    #[must_use]
    pub fn of(&self, node: usize) -> &[u32] {
        &self.entries[self.offsets[node] as usize..self.offsets[node + 1] as usize]
    }
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
    /// A node's constraint directions are not unit, orthogonal, or zero.
    #[error("node {node}'s constraint is invalid: {reason}")]
    InvalidConstraint {
        /// The node.
        node: usize,
        /// What is wrong.
        reason: &'static str,
    },
    /// The model has more elements or surface triangles than a `u32` slot
    /// index can address.
    #[error("the model is too large for u32 slot indices")]
    TooLarge,
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
        let element_incidence = Incidence::new(&elements, nodes)?;
        let surface_incidence = Incidence::new(&surface, nodes)?;

        Ok(Self {
            rest_positions,
            elements,
            materials,
            held,
            constraints: vec![[[0.0; 3]; 2]; nodes],
            rest_edge_inverses,
            rest_volumes,
            node_masses,
            node_rest_volumes,
            node_lambdas,
            surface,
            element_incidence,
            surface_incidence,
        })
    }

    /// Add per-direction constraints: for each node, two directions its
    /// displacement and velocity have no component along (plan §16d).
    ///
    /// Each direction is a unit vector or zero (unused), and two used ones
    /// are orthogonal. They are fixed at rest, so a constraint is a plane the
    /// node moves in. A node held whole stays `held`.
    ///
    /// # Errors
    /// [`ModelError::LengthMismatch`] if there is not one pair per node, and
    /// [`ModelError::InvalidConstraint`] naming the first bad node.
    pub fn with_constraints(mut self, constraints: Vec<[[f64; 3]; 2]>) -> Result<Self, ModelError> {
        check_length("constraints", constraints.len(), self.node_count())?;
        for (node, [first, second]) in constraints.iter().enumerate() {
            let reason = if ![first, second]
                .iter()
                .all(|d| d.iter().all(|c| c.is_finite()))
            {
                Some("a direction is not finite")
            } else if ![first, second].iter().all(|d| {
                let squared = dot(**d, **d);
                squared == 0.0 || (squared - 1.0).abs() <= CONSTRAINT_TOLERANCE
            }) {
                Some("a direction is neither zero nor unit length")
            } else if dot(*first, *second).abs() > CONSTRAINT_TOLERANCE {
                Some("the two directions are not orthogonal")
            } else {
                None
            };
            if let Some(reason) = reason {
                return Err(ModelError::InvalidConstraint { node, reason });
            }
        }
        self.constraints = constraints;
        Ok(self)
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

    /// Each node's two constraint directions, zero where unused
    /// ([`ExplicitModel::with_constraints`]).
    #[must_use]
    pub fn constraints(&self) -> &[[[f64; 3]; 2]] {
        &self.constraints
    }

    /// For each node, the element slots that refer to it: `element · 4 +
    /// corner`. The element-to-node gathers walk it.
    #[must_use]
    pub const fn element_incidence(&self) -> &Incidence {
        &self.element_incidence
    }

    /// For each node, the surface-triangle slots that refer to it:
    /// `triangle · 3 + corner`, into [`ExplicitModel::surface_triangles`].
    /// The pressure readout's tributary areas are gathered through it.
    #[must_use]
    pub const fn surface_incidence(&self) -> &Incidence {
        &self.surface_incidence
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

/// How far from unit length, or from orthogonal, a constraint direction may
/// be: rounding in the caller's normalization, nothing more.
const CONSTRAINT_TOLERANCE: f64 = 1e-12;

const fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
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
    let reason = if ![m.mu, m.lambda, m.c2, m.viscosity, m.density]
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
    } else if m.viscosity < 0.0 {
        Some("the viscosity must not be negative")
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
