//! Core types for lattice generation.

use crate::beam::BeamLatticeData;
use mesh_types::{IndexedMesh, MeshTopology};

/// Types of lattice structures that can be generated.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum LatticeType {
    /// Simple cubic grid pattern.
    ///
    /// Struts along X, Y, Z axes connecting grid vertices.
    /// Easy to print, moderate strength.
    #[default]
    Cubic,

    /// Octet-truss lattice.
    ///
    /// Combines octahedron and tetrahedron structures.
    /// High strength-to-weight ratio, commonly used in aerospace.
    OctetTruss,

    /// Gyroid TPMS (triply periodic minimal surface).
    ///
    /// Smooth, self-supporting structure with no flat overhangs.
    /// Excellent for FDM printing without supports.
    Gyroid,

    /// Schwarz-P TPMS surface.
    ///
    /// Primitive surface with cubic symmetry.
    /// Good isotropic mechanical properties.
    SchwarzP,

    /// Diamond TPMS surface.
    ///
    /// Good mechanical isotropy and strength distribution.
    Diamond,

    /// Voronoi-based lattice.
    ///
    /// Organic, irregular cell structure.
    /// Currently implemented as perturbed cubic (simplified).
    Voronoi,
}

impl LatticeType {
    /// Returns the name of this lattice type.
    #[must_use]
    pub const fn name(&self) -> &'static str {
        match self {
            Self::Cubic => "Cubic",
            Self::OctetTruss => "Octet-Truss",
            Self::Gyroid => "Gyroid",
            Self::SchwarzP => "Schwarz-P",
            Self::Diamond => "Diamond",
            Self::Voronoi => "Voronoi",
        }
    }

    /// Returns true if this lattice type uses TPMS (implicit surface).
    #[must_use]
    pub const fn is_tpms(&self) -> bool {
        matches!(self, Self::Gyroid | Self::SchwarzP | Self::Diamond)
    }

    /// Returns true if this lattice type uses struts (explicit geometry).
    #[must_use]
    pub const fn is_strut_based(&self) -> bool {
        matches!(self, Self::Cubic | Self::OctetTruss | Self::Voronoi)
    }

    /// Returns the recommended resolution for TPMS lattices.
    ///
    /// Higher resolution produces smoother surfaces but more triangles.
    #[must_use]
    pub const fn recommended_resolution(&self) -> usize {
        match self {
            Self::Gyroid | Self::SchwarzP | Self::Diamond => 15,
            _ => 10,
        }
    }
}

/// Result of lattice generation.
#[derive(Debug, Clone)]
pub struct LatticeResult {
    /// The generated lattice mesh.
    pub mesh: IndexedMesh,

    /// Actual volume fraction achieved (0.0 to 1.0).
    pub actual_density: f64,

    /// Number of unit cells in the lattice.
    pub cell_count: usize,

    /// Total length of all struts (for strut-based lattices).
    ///
    /// This is `None` for TPMS lattices.
    pub total_strut_length: Option<f64>,

    /// Raw beam data for 3MF export.
    ///
    /// Only populated if `preserve_beam_data` is enabled in params.
    pub beam_data: Option<BeamLatticeData>,
}

impl LatticeResult {
    /// Creates a new lattice result.
    #[must_use]
    pub const fn new(mesh: IndexedMesh, actual_density: f64, cell_count: usize) -> Self {
        Self {
            mesh,
            actual_density,
            cell_count,
            total_strut_length: None,
            beam_data: None,
        }
    }

    /// Sets the total strut length.
    #[must_use]
    pub const fn with_strut_length(mut self, length: f64) -> Self {
        self.total_strut_length = Some(length);
        self
    }

    /// Sets the beam data for 3MF export.
    #[must_use]
    pub fn with_beam_data(mut self, data: BeamLatticeData) -> Self {
        self.beam_data = Some(data);
        self
    }

    /// Returns the number of vertices in the lattice mesh.
    #[must_use]
    pub fn vertex_count(&self) -> usize {
        self.mesh.vertex_count()
    }

    /// Returns the number of triangles in the lattice mesh.
    #[must_use]
    pub fn triangle_count(&self) -> usize {
        self.mesh.face_count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lattice_type_name() {
        assert_eq!(LatticeType::Cubic.name(), "Cubic");
        assert_eq!(LatticeType::OctetTruss.name(), "Octet-Truss");
        assert_eq!(LatticeType::Gyroid.name(), "Gyroid");
        assert_eq!(LatticeType::SchwarzP.name(), "Schwarz-P");
        assert_eq!(LatticeType::Diamond.name(), "Diamond");
        assert_eq!(LatticeType::Voronoi.name(), "Voronoi");
    }

    #[test]
    fn test_lattice_type_is_tpms() {
        assert!(!LatticeType::Cubic.is_tpms());
        assert!(!LatticeType::OctetTruss.is_tpms());
        assert!(LatticeType::Gyroid.is_tpms());
        assert!(LatticeType::SchwarzP.is_tpms());
        assert!(LatticeType::Diamond.is_tpms());
        assert!(!LatticeType::Voronoi.is_tpms());
    }

    #[test]
    fn test_lattice_type_is_strut_based() {
        assert!(LatticeType::Cubic.is_strut_based());
        assert!(LatticeType::OctetTruss.is_strut_based());
        assert!(!LatticeType::Gyroid.is_strut_based());
        assert!(!LatticeType::SchwarzP.is_strut_based());
        assert!(!LatticeType::Diamond.is_strut_based());
        assert!(LatticeType::Voronoi.is_strut_based());
    }

    #[test]
    fn test_lattice_type_default() {
        assert_eq!(LatticeType::default(), LatticeType::Cubic);
    }

    #[test]
    fn test_lattice_result_new() {
        let mesh = IndexedMesh::new();
        let result = LatticeResult::new(mesh, 0.25, 100);
        assert!((result.actual_density - 0.25).abs() < f64::EPSILON);
        assert_eq!(result.cell_count, 100);
        assert!(result.total_strut_length.is_none());
        assert!(result.beam_data.is_none());
    }

    #[test]
    fn test_lattice_result_with_strut_length() {
        let mesh = IndexedMesh::new();
        let result = LatticeResult::new(mesh, 0.3, 50).with_strut_length(1234.5);
        assert_eq!(result.total_strut_length, Some(1234.5));
    }
}
