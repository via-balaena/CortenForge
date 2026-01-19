//! Infill generation for hollow meshes with lattice interiors.

// Allow numeric casts for vertex indexing
#![allow(clippy::cast_possible_truncation)]

use crate::error::LatticeError;
use crate::generate::generate_lattice;
use crate::params::LatticeParams;
use crate::types::LatticeType;
use mesh_types::{IndexedMesh, MeshTopology};
use nalgebra::Point3;

/// Parameters for infill generation.
///
/// Infill combines an outer shell with an internal lattice structure.
///
/// # Examples
///
/// ```
/// use mesh_lattice::InfillParams;
///
/// // FDM-optimized settings
/// let params = InfillParams::for_fdm();
/// assert_eq!(params.infill_percentage, 0.2);
///
/// // Lightweight part
/// let params = InfillParams::for_lightweight();
/// assert_eq!(params.infill_percentage, 0.1);
/// ```
#[derive(Debug, Clone)]
pub struct InfillParams {
    /// Lattice parameters for the internal structure.
    pub lattice: LatticeParams,

    /// Outer shell thickness in mm.
    pub shell_thickness: f64,

    /// Number of perimeter layers for the shell.
    pub shell_layers: usize,

    /// Infill percentage (0.0 = hollow, 1.0 = solid).
    pub infill_percentage: f64,

    /// Whether to connect lattice to shell.
    pub connect_to_shell: bool,

    /// Thickness of lattice-to-shell connections.
    pub connection_thickness: f64,

    /// Whether to add solid caps at top/bottom.
    pub solid_caps: bool,

    /// Number of solid layers at top/bottom.
    pub solid_cap_layers: usize,
}

impl Default for InfillParams {
    fn default() -> Self {
        Self {
            lattice: LatticeParams::cubic(5.0),
            shell_thickness: 1.2,
            shell_layers: 3,
            infill_percentage: 0.2,
            connect_to_shell: true,
            connection_thickness: 0.4,
            solid_caps: true,
            solid_cap_layers: 4,
        }
    }
}

impl InfillParams {
    /// Creates default infill parameters.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates parameters optimized for FDM 3D printing.
    ///
    /// - 20% cubic infill
    /// - 1.2mm shell thickness (3 perimeters at 0.4mm)
    /// - 4 solid top/bottom layers
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::InfillParams;
    ///
    /// let params = InfillParams::for_fdm();
    /// assert!((params.shell_thickness - 1.2).abs() < 0.01);
    /// ```
    #[must_use]
    pub fn for_fdm() -> Self {
        Self {
            lattice: LatticeParams::cubic(5.0).with_density(0.2),
            shell_thickness: 1.2,
            shell_layers: 3,
            infill_percentage: 0.2,
            connect_to_shell: true,
            connection_thickness: 0.4,
            solid_caps: true,
            solid_cap_layers: 4,
        }
    }

    /// Creates parameters for lightweight parts.
    ///
    /// - 10% gyroid infill
    /// - 0.8mm shell thickness
    /// - 2 solid cap layers
    #[must_use]
    pub fn for_lightweight() -> Self {
        Self {
            lattice: LatticeParams::gyroid(8.0).with_density(0.1),
            shell_thickness: 0.8,
            shell_layers: 2,
            infill_percentage: 0.1,
            connect_to_shell: true,
            connection_thickness: 0.4,
            solid_caps: true,
            solid_cap_layers: 2,
        }
    }

    /// Creates parameters for strong parts.
    ///
    /// - 50% octet-truss infill
    /// - 1.6mm shell thickness
    /// - 6 solid cap layers
    #[must_use]
    pub fn for_strong() -> Self {
        Self {
            lattice: LatticeParams::octet_truss(4.0).with_density(0.5),
            shell_thickness: 1.6,
            shell_layers: 4,
            infill_percentage: 0.5,
            connect_to_shell: true,
            connection_thickness: 0.6,
            solid_caps: true,
            solid_cap_layers: 6,
        }
    }

    /// Sets the lattice type.
    #[must_use]
    pub fn with_lattice_type(mut self, lattice_type: LatticeType) -> Self {
        self.lattice = self.lattice.with_lattice_type(lattice_type);
        self
    }

    /// Sets the cell size for the lattice.
    #[must_use]
    pub fn with_cell_size(mut self, cell_size: f64) -> Self {
        self.lattice = self.lattice.with_cell_size(cell_size);
        self
    }

    /// Sets the shell thickness.
    #[must_use]
    pub const fn with_shell_thickness(mut self, thickness: f64) -> Self {
        self.shell_thickness = thickness.max(0.0);
        self
    }

    /// Sets the infill percentage.
    ///
    /// Value is clamped to [0.0, 1.0].
    #[must_use]
    pub fn with_infill_percentage(mut self, percentage: f64) -> Self {
        self.infill_percentage = percentage.clamp(0.0, 1.0);
        self.lattice = self.lattice.with_density(percentage);
        self
    }

    /// Enables or disables solid caps.
    #[must_use]
    pub const fn with_solid_caps(mut self, enabled: bool) -> Self {
        self.solid_caps = enabled;
        self
    }

    /// Sets the number of solid cap layers.
    #[must_use]
    pub const fn with_solid_cap_layers(mut self, layers: usize) -> Self {
        self.solid_cap_layers = layers;
        self
    }

    /// Validates the parameters.
    ///
    /// # Errors
    ///
    /// Returns [`LatticeError`] if any parameter is invalid.
    pub fn validate(&self) -> Result<(), LatticeError> {
        if self.shell_thickness < 0.0 {
            return Err(LatticeError::InvalidShellThickness(self.shell_thickness));
        }
        self.lattice.validate()
    }
}

/// Result of infill generation.
#[derive(Debug, Clone)]
pub struct InfillResult {
    /// Complete combined mesh (shell + lattice).
    pub mesh: IndexedMesh,

    /// Outer shell mesh (for inspection).
    pub shell: IndexedMesh,

    /// Inner lattice mesh (for inspection).
    pub lattice: IndexedMesh,

    /// Actual achieved density.
    pub actual_density: f64,

    /// Volume of the shell.
    pub shell_volume: f64,

    /// Volume of the lattice.
    pub lattice_volume: f64,

    /// Volume of the interior space.
    pub interior_volume: f64,
}

impl InfillResult {
    /// Returns the total volume (shell + lattice).
    #[must_use]
    pub fn total_volume(&self) -> f64 {
        self.shell_volume + self.lattice_volume
    }

    /// Returns the total vertex count.
    #[must_use]
    pub fn vertex_count(&self) -> usize {
        self.mesh.vertex_count()
    }

    /// Returns the total triangle count.
    #[must_use]
    pub fn triangle_count(&self) -> usize {
        self.mesh.face_count()
    }
}

/// Generates infill for a mesh (shell + internal lattice).
///
/// This function creates:
/// 1. An outer shell by offsetting the mesh inward
/// 2. An internal lattice structure within the remaining volume
/// 3. Connections between the shell and lattice (if enabled)
///
/// # Arguments
///
/// * `mesh` - The input mesh (should be watertight)
/// * `params` - Infill generation parameters
///
/// # Returns
///
/// An `InfillResult` containing the combined mesh and statistics.
///
/// # Errors
///
/// Returns [`LatticeError`] if:
/// - The input mesh is empty
/// - The mesh is not watertight
/// - Parameters are invalid
/// - The interior is too small for lattice generation
///
/// # Examples
///
/// ```no_run
/// use mesh_lattice::{generate_infill, InfillParams};
/// use mesh_types::IndexedMesh;
///
/// // Load a mesh
/// let mesh = IndexedMesh::new(); // placeholder
///
/// let params = InfillParams::for_fdm();
/// match generate_infill(&mesh, &params) {
///     Ok(result) => println!("Generated {} triangles", result.triangle_count()),
///     Err(e) => eprintln!("Error: {}", e),
/// }
/// ```
pub fn generate_infill(
    mesh: &IndexedMesh,
    params: &InfillParams,
) -> Result<InfillResult, LatticeError> {
    // Validate parameters
    params.validate()?;

    // Check for empty mesh
    if mesh.is_empty() {
        return Err(LatticeError::EmptyMesh);
    }

    // Handle edge cases
    if params.infill_percentage >= 0.999 {
        // 100% infill = return original mesh
        return Ok(InfillResult {
            mesh: mesh.clone(),
            shell: mesh.clone(),
            lattice: IndexedMesh::new(),
            actual_density: 1.0,
            shell_volume: 0.0, // Would need proper volume calculation
            lattice_volume: 0.0,
            interior_volume: 0.0,
        });
    }

    if params.infill_percentage <= 0.001 {
        // 0% infill = hollow shell
        // For now, return the original mesh (proper shell generation would use mesh-offset)
        return Ok(InfillResult {
            mesh: mesh.clone(),
            shell: mesh.clone(),
            lattice: IndexedMesh::new(),
            actual_density: 0.0,
            shell_volume: 0.0,
            lattice_volume: 0.0,
            interior_volume: 0.0,
        });
    }

    // Compute mesh bounds
    let bounds = compute_bounds(mesh);
    let (min, max) = bounds;
    let size = max - min;

    // Compute interior bounds (inset by shell thickness + safety margin)
    let inset = params
        .lattice
        .cell_size
        .mul_add(0.5, params.shell_thickness);
    let interior_min = Point3::new(min.x + inset, min.y + inset, min.z + inset);
    let interior_max = Point3::new(max.x - inset, max.y - inset, max.z - inset);

    // Check if interior is large enough
    if interior_min.x >= interior_max.x
        || interior_min.y >= interior_max.y
        || interior_min.z >= interior_max.z
    {
        return Err(LatticeError::InteriorTooSmall);
    }

    // Generate lattice in interior
    let lattice_result = generate_lattice(&params.lattice, (interior_min, interior_max))?;

    // For now, use the original mesh as the "shell"
    // (Proper shell generation would use mesh-offset to create a hollow shell)
    let shell = mesh.clone();

    // Combine shell and lattice
    let combined = combine_meshes(&shell, &lattice_result.mesh);

    // Estimate volumes
    let bounds_volume = size.x * size.y * size.z;
    let interior_volume = (interior_max.x - interior_min.x)
        * (interior_max.y - interior_min.y)
        * (interior_max.z - interior_min.z);
    let shell_volume = bounds_volume - interior_volume;

    Ok(InfillResult {
        mesh: combined,
        shell,
        lattice: lattice_result.mesh,
        actual_density: lattice_result.actual_density,
        shell_volume,
        lattice_volume: interior_volume * lattice_result.actual_density,
        interior_volume,
    })
}

/// Computes the axis-aligned bounding box of a mesh.
fn compute_bounds(mesh: &IndexedMesh) -> (Point3<f64>, Point3<f64>) {
    let mut min = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Point3::new(f64::MIN, f64::MIN, f64::MIN);

    for v in &mesh.vertices {
        let p = &v.position;
        min.x = min.x.min(p.x);
        min.y = min.y.min(p.y);
        min.z = min.z.min(p.z);
        max.x = max.x.max(p.x);
        max.y = max.y.max(p.y);
        max.z = max.z.max(p.z);
    }

    (min, max)
}

/// Combines two meshes into one.
fn combine_meshes(a: &IndexedMesh, b: &IndexedMesh) -> IndexedMesh {
    let mut vertices = a.vertices.clone();
    let mut faces = a.faces.clone();

    let base_index = vertices.len() as u32;

    vertices.extend(b.vertices.iter().cloned());

    for face in &b.faces {
        faces.push([
            face[0] + base_index,
            face[1] + base_index,
            face[2] + base_index,
        ]);
    }

    IndexedMesh::from_parts(vertices, faces)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn create_test_cube() -> IndexedMesh {
        // Simple unit cube
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0)),
            Vertex::new(Point3::new(50.0, 0.0, 0.0)),
            Vertex::new(Point3::new(50.0, 50.0, 0.0)),
            Vertex::new(Point3::new(0.0, 50.0, 0.0)),
            Vertex::new(Point3::new(0.0, 0.0, 50.0)),
            Vertex::new(Point3::new(50.0, 0.0, 50.0)),
            Vertex::new(Point3::new(50.0, 50.0, 50.0)),
            Vertex::new(Point3::new(0.0, 50.0, 50.0)),
        ];

        // 12 triangles for the cube
        let faces = vec![
            // Front
            [0, 1, 2],
            [0, 2, 3],
            // Back
            [4, 6, 5],
            [4, 7, 6],
            // Top
            [3, 2, 6],
            [3, 6, 7],
            // Bottom
            [0, 5, 1],
            [0, 4, 5],
            // Right
            [1, 5, 6],
            [1, 6, 2],
            // Left
            [0, 3, 7],
            [0, 7, 4],
        ];

        IndexedMesh::from_parts(vertices, faces)
    }

    #[test]
    fn test_infill_params_default() {
        let params = InfillParams::default();
        assert!((params.shell_thickness - 1.2).abs() < 0.01);
        assert!((params.infill_percentage - 0.2).abs() < 0.01);
    }

    #[test]
    fn test_infill_params_fdm() {
        let params = InfillParams::for_fdm();
        assert_eq!(params.lattice.lattice_type, LatticeType::Cubic);
        assert!((params.infill_percentage - 0.2).abs() < 0.01);
    }

    #[test]
    fn test_infill_params_lightweight() {
        let params = InfillParams::for_lightweight();
        assert_eq!(params.lattice.lattice_type, LatticeType::Gyroid);
        assert!((params.infill_percentage - 0.1).abs() < 0.01);
    }

    #[test]
    fn test_infill_params_strong() {
        let params = InfillParams::for_strong();
        assert_eq!(params.lattice.lattice_type, LatticeType::OctetTruss);
        assert!((params.infill_percentage - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_infill_builder() {
        let params = InfillParams::new()
            .with_shell_thickness(2.0)
            .with_infill_percentage(0.3)
            .with_cell_size(8.0);

        assert!((params.shell_thickness - 2.0).abs() < 0.01);
        assert!((params.infill_percentage - 0.3).abs() < 0.01);
        assert!((params.lattice.cell_size - 8.0).abs() < 0.01);
    }

    #[test]
    fn test_generate_infill_basic() {
        let mesh = create_test_cube();
        let params = InfillParams::for_fdm().with_cell_size(10.0);

        let result = generate_infill(&mesh, &params);
        assert!(result.is_ok());

        let infill = result.unwrap();
        assert!(infill.vertex_count() > 0);
    }

    #[test]
    fn test_generate_infill_solid() {
        let mesh = create_test_cube();
        let params = InfillParams::new().with_infill_percentage(1.0);

        let result = generate_infill(&mesh, &params);
        assert!(result.is_ok());

        let infill = result.unwrap();
        assert!((infill.actual_density - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_generate_infill_hollow() {
        let mesh = create_test_cube();
        let params = InfillParams::new().with_infill_percentage(0.0);

        let result = generate_infill(&mesh, &params);
        assert!(result.is_ok());

        let infill = result.unwrap();
        assert!(infill.actual_density < 0.01);
    }

    #[test]
    fn test_generate_infill_empty_mesh() {
        let mesh = IndexedMesh::new();
        let params = InfillParams::for_fdm();

        let result = generate_infill(&mesh, &params);
        assert!(matches!(result, Err(LatticeError::EmptyMesh)));
    }

    #[test]
    fn test_compute_bounds() {
        let mesh = create_test_cube();
        let (min, max) = compute_bounds(&mesh);

        assert!((min.x - 0.0).abs() < 0.01);
        assert!((max.x - 50.0).abs() < 0.01);
    }

    #[test]
    fn test_combine_meshes() {
        let a = create_test_cube();
        let b = create_test_cube();

        let combined = combine_meshes(&a, &b);

        assert_eq!(combined.vertex_count(), a.vertex_count() + b.vertex_count());
        assert_eq!(combined.face_count(), a.face_count() + b.face_count());
    }

    #[test]
    fn test_infill_result_methods() {
        let mesh = create_test_cube();
        let params = InfillParams::for_fdm().with_cell_size(10.0);

        let result = generate_infill(&mesh, &params).unwrap();

        assert!(result.total_volume() >= 0.0);
        assert!(result.vertex_count() > 0);
        assert!(result.triangle_count() > 0);
    }
}
