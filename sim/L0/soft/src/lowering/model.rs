//! The model: a meshed wall as the explicit solver's [`ExplicitModel`] (plan §16j, §16w).

use sim_soft_explicit::f64::Material;
use sim_soft_explicit::{ExplicitModel, ModelError};

use crate::Yeoh;
use crate::mesh::{Mesh, TetId, VertexId};
use crate::sdf_bridge::SdfMeshedTetMesh;

/// A wall lowered into the explicit solver's model.
#[derive(Clone, Debug)]
pub struct Lowered {
    /// The model.
    pub model: ExplicitModel,
    /// Each model node's vertex in the mesh. The mesher leaves lattice vertices that no element names; the model
    /// has no massless nodes, so it drops them.
    pub source: Vec<VertexId>,
}

/// What the lowering sets that the mesh does not carry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Lowering {
    /// Poisson's ratio: each element's λ is its μ's at this ratio. The silicone catalog's λ is 4μ (ν 0.4), which the
    /// explicit solver does not use (plan §5b, §16j).
    pub poisson: f64,
    /// The Kelvin–Voigt viscosity over the shear modulus, `η/μ` (plan §16p): each element's `η` is its `μ` times
    /// this. 0 is elastic.
    pub viscous_time: f64,
}

/// Why a wall cannot be lowered.
#[derive(Clone, Debug, PartialEq)]
pub enum LoweringError {
    /// Poisson's ratio is not in `[0, 0.5)`, or the viscous time is negative or not finite.
    Parameters,
    /// Not one density per element.
    Densities {
        /// How many were given.
        found: usize,
        /// How many elements the mesh has.
        expected: usize,
    },
    /// A held vertex is not one the elements name.
    Held {
        /// The vertex.
        vertex: VertexId,
    },
    /// Two materials meet in the wall. Where they meet, the rule for each node's pressure is provisional until it
    /// is chosen against a two-layer reference, which moved to a PR of its own (plan §15g step 1, §16w).
    MaterialsMeet {
        /// The first element whose material differs from the first element's.
        element: usize,
    },
    /// The model refused the lowered data.
    Model(ModelError),
}

impl std::fmt::Display for LoweringError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Parameters => write!(
                f,
                "Poisson's ratio must lie in [0, 0.5) and the viscous time be finite and not negative"
            ),
            Self::Densities { found, expected } => {
                write!(f, "{found} densities for {expected} elements")
            }
            Self::Held { vertex } => write!(f, "held vertex {vertex} is not one the elements name"),
            Self::MaterialsMeet { element } => write!(
                f,
                "element {element}'s material differs from the first's: where materials meet, the pressure rule \
                 is not chosen yet (plan §16w)"
            ),
            Self::Model(error) => write!(f, "the model refused the lowered data: {error}"),
        }
    }
}

impl std::error::Error for LoweringError {}

/// Lower `mesh` into an [`ExplicitModel`], with the vertices `held` held whole.
///
/// The model keeps the vertices the elements name and the elements as the mesher orders them (positive rest
/// volume by construction; the model refuses any other). Per element it takes the mesher's `μ` and `C₂`, `λ` at
/// [`Lowering::poisson`], `η` as [`Lowering::viscous_time`] times `μ`, and `densities[element]`.
///
/// # Errors
/// [`LoweringError`]: the lowering's numbers are out of range, there is not one density per element, a held vertex
/// is not one the elements name, the model refuses the data (a material that is not finite, among others), or two
/// materials meet.
pub fn lower(
    mesh: &SdfMeshedTetMesh<Yeoh>,
    densities: &[f64],
    lowering: Lowering,
    held: &[VertexId],
) -> Result<Lowered, LoweringError> {
    let Lowering {
        poisson,
        viscous_time,
    } = lowering;
    if !((0.0..0.5).contains(&poisson) && viscous_time.is_finite() && viscous_time >= 0.0) {
        return Err(LoweringError::Parameters);
    }
    if densities.len() != mesh.n_tets() {
        return Err(LoweringError::Densities {
            found: densities.len(),
            expected: mesh.n_tets(),
        });
    }
    let positions = mesh.positions();
    let mut index = vec![None; positions.len()];
    let mut source = Vec::new();
    let mut elements = Vec::with_capacity(mesh.n_tets());
    let mut materials = Vec::with_capacity(mesh.n_tets());
    let too_large = || LoweringError::Model(ModelError::TooLarge);
    for (element, (yeoh, &density)) in mesh.materials().iter().zip(densities).enumerate() {
        let tet = TetId::try_from(element).map_err(|_| too_large())?;
        let mut lowered = [0_u32; 4];
        for (slot, vertex) in lowered.iter_mut().zip(mesh.tet_vertices(tet)) {
            if let Some(node) = index[vertex as usize] {
                *slot = node;
                continue;
            }
            let node = u32::try_from(source.len()).map_err(|_| too_large())?;
            index[vertex as usize] = Some(node);
            source.push(vertex);
            *slot = node;
        }
        elements.push(lowered);
        materials.push(Material {
            mu: yeoh.mu(),
            lambda: yeoh.mu() * 2.0 * poisson / (1.0 - 2.0 * poisson),
            c2: yeoh.c2(),
            viscosity: viscous_time * yeoh.mu(),
            density,
        });
    }
    let mut held_nodes = vec![false; source.len()];
    for &vertex in held {
        let node = index
            .get(vertex as usize)
            .copied()
            .flatten()
            .ok_or(LoweringError::Held { vertex })?;
        held_nodes[node as usize] = true;
    }
    let rest_positions = source
        .iter()
        .map(|&v| {
            let p = positions[v as usize];
            [p.x, p.y, p.z]
        })
        .collect();
    // Built first, the model refuses a material that is not finite, so the check below compares valid materials only.
    let model = ExplicitModel::new(rest_positions, elements, materials, held_nodes)
        .map_err(LoweringError::Model)?;
    let materials = model.materials();
    if let Some(element) = materials.iter().position(|m| *m != materials[0]) {
        return Err(LoweringError::MaterialsMeet { element });
    }
    Ok(Lowered { model, source })
}
