//! 3D deformable bodies: soft body simulation with tetrahedral meshes.
//!
//! Soft bodies are volumetric objects that can deform under forces.
//! They are represented as tetrahedral meshes with:
//!
//! - **Distance constraints**: Along edges to resist stretching
//! - **Volume constraints**: Per tetrahedron to resist compression/expansion
//!
//! # Structure
//!
//! ```text
//!       ●
//!      /|\
//!     / | \
//!    ●──●──●
//!     \ | /
//!      \|/
//!       ●
//! ```
//!
//! Each tetrahedron has:
//! - 4 vertices
//! - 6 edges (distance constraints)
//! - 1 volume constraint

use hashbrown::HashSet;
use nalgebra::{Point3, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::DeformableBody;
use crate::constraints::{Constraint, DistanceConstraint, VolumeConstraint};
use crate::material::{Material, MaterialPreset};
use crate::mesh::Tetrahedron;
use crate::types::{DeformableId, VertexFlags, next_deformable_id};

/// Configuration for soft body simulation.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SoftBodyConfig {
    /// Density in kg/m³.
    pub density: f64,
    /// Distance compliance (0 = rigid edges).
    pub distance_compliance: f64,
    /// Volume compliance (0 = incompressible).
    pub volume_compliance: f64,
    /// Damping coefficient.
    pub damping: f64,
}

impl Default for SoftBodyConfig {
    fn default() -> Self {
        Self {
            density: 1000.0,           // Water-like density
            distance_compliance: 1e-6, // Somewhat stiff
            volume_compliance: 1e-6,   // Volume preserving
            damping: 0.02,
        }
    }
}

impl SoftBodyConfig {
    /// Create a config for rubber-like material.
    #[must_use]
    pub const fn rubber() -> Self {
        Self {
            density: 1100.0,
            distance_compliance: 1e-5,
            volume_compliance: 1e-8, // Nearly incompressible
            damping: 0.05,
        }
    }

    /// Create a config for gelatin/jelly.
    #[must_use]
    pub const fn gelatin() -> Self {
        Self {
            density: 1050.0,
            distance_compliance: 1e-4,
            volume_compliance: 1e-6,
            damping: 0.1,
        }
    }

    /// Create a config for soft tissue.
    #[must_use]
    pub const fn soft_tissue() -> Self {
        Self {
            density: 1060.0,
            distance_compliance: 1e-5,
            volume_compliance: 1e-7,
            damping: 0.15,
        }
    }

    /// Create a config for muscle.
    #[must_use]
    pub const fn muscle() -> Self {
        Self {
            density: 1050.0,
            distance_compliance: 5e-6,
            volume_compliance: 1e-8, // Nearly incompressible
            damping: 0.1,
        }
    }

    /// Create a config for foam/sponge.
    #[must_use]
    pub const fn foam() -> Self {
        Self {
            density: 50.0, // Very light
            distance_compliance: 1e-4,
            volume_compliance: 1e-3, // Compressible
            damping: 0.2,
        }
    }

    /// Create a config for stiff material.
    #[must_use]
    pub const fn stiff() -> Self {
        Self {
            density: 2000.0,
            distance_compliance: 1e-8,
            volume_compliance: 1e-8,
            damping: 0.01,
        }
    }
}

/// A 3D deformable body using tetrahedral mesh.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SoftBody {
    /// Unique identifier.
    id: DeformableId,
    /// Name.
    name: String,
    /// Vertex positions.
    positions: Vec<Point3<f64>>,
    /// Vertex velocities.
    velocities: Vec<Vector3<f64>>,
    /// Inverse masses.
    inv_masses: Vec<f64>,
    /// Original masses.
    masses: Vec<f64>,
    /// Vertex flags.
    vertex_flags: Vec<VertexFlags>,
    /// External forces.
    external_forces: Vec<Vector3<f64>>,
    /// Constraints.
    constraints: Vec<Constraint>,
    /// Tetrahedra.
    tetrahedra: Vec<Tetrahedron>,
    /// Surface triangles (for collision/rendering).
    surface_triangles: Vec<[usize; 3]>,
    /// Material.
    material: Material,
    /// Configuration.
    config: SoftBodyConfig,
}

impl SoftBody {
    /// Create a soft body from vertices and tetrahedra.
    ///
    /// # Arguments
    ///
    /// * `name` - Name of the soft body
    /// * `positions` - Vertex positions
    /// * `tetrahedra` - Tetrahedral elements
    /// * `config` - Configuration
    #[must_use]
    pub fn from_tetrahedra(
        name: &str,
        positions: Vec<Point3<f64>>,
        tetrahedra: Vec<Tetrahedron>,
        config: SoftBodyConfig,
    ) -> Self {
        let num_vertices = positions.len();

        // Compute masses based on tetrahedron volumes
        let mut masses = vec![0.0; num_vertices];
        for tet in &tetrahedra {
            let volume = tet.volume(&positions);
            let mass_per_vertex = (config.density * volume) / 4.0;

            for &v in &tet.vertices {
                masses[v] += mass_per_vertex;
            }
        }

        // Ensure minimum mass
        for m in &mut masses {
            if *m < 1e-10 {
                *m = 0.001;
            }
        }

        let inv_masses: Vec<f64> = masses.iter().map(|&m| 1.0 / m).collect();

        // Create constraints
        let mut constraints = Vec::new();

        // Distance constraints for unique edges
        let mut edges_added: HashSet<(usize, usize)> = HashSet::new();
        for tet in &tetrahedra {
            for edge in tet.edges() {
                let key = if edge.v0 < edge.v1 {
                    (edge.v0, edge.v1)
                } else {
                    (edge.v1, edge.v0)
                };

                if !edges_added.contains(&key) {
                    edges_added.insert(key);
                    let rest_length = (positions[edge.v1] - positions[edge.v0]).norm();
                    constraints.push(Constraint::Distance(DistanceConstraint::new(
                        edge.v0,
                        edge.v1,
                        rest_length,
                        config.distance_compliance,
                    )));
                }
            }
        }

        // Volume constraints for each tetrahedron
        for tet in &tetrahedra {
            let rest_volume = tet.volume(&positions);
            constraints.push(Constraint::Volume(VolumeConstraint::new(
                tet.vertices,
                rest_volume,
                config.volume_compliance,
            )));
        }

        // Extract surface triangles
        let surface_triangles = Self::extract_surface_triangles(&tetrahedra);

        let material = Material::preset(MaterialPreset::SoftTissue).with_damping(config.damping);

        Self {
            id: next_deformable_id(),
            name: name.to_string(),
            positions,
            velocities: vec![Vector3::zeros(); num_vertices],
            inv_masses,
            masses,
            vertex_flags: vec![VertexFlags::empty(); num_vertices],
            external_forces: vec![Vector3::zeros(); num_vertices],
            constraints,
            tetrahedra,
            surface_triangles,
            material,
            config,
        }
    }

    /// Extract boundary triangles from tetrahedra.
    fn extract_surface_triangles(tetrahedra: &[Tetrahedron]) -> Vec<[usize; 3]> {
        use hashbrown::HashMap;

        // Count occurrences of each triangle
        let mut face_count: HashMap<[usize; 3], usize> = HashMap::new();

        for tet in tetrahedra {
            for face in tet.faces() {
                // Canonicalize the face (sort indices)
                let mut indices = face.vertices;
                indices.sort_unstable();
                *face_count.entry(indices).or_insert(0) += 1;
            }
        }

        // Surface triangles appear exactly once
        face_count
            .into_iter()
            .filter(|(_, count)| *count == 1)
            .map(|(indices, _)| indices)
            .collect()
    }

    /// Create a simple cube soft body.
    ///
    /// # Arguments
    ///
    /// * `name` - Name
    /// * `center` - Center position
    /// * `size` - Half-size of the cube
    /// * `subdivisions` - Number of subdivisions per axis (0 = single cube)
    /// * `config` - Configuration
    #[must_use]
    pub fn cube(
        name: &str,
        center: Point3<f64>,
        size: f64,
        subdivisions: usize,
        config: SoftBodyConfig,
    ) -> Self {
        let n = subdivisions + 2; // Number of vertices per axis
        let step = 2.0 * size / (n - 1) as f64;

        // Create vertices
        let mut positions = Vec::with_capacity(n * n * n);
        for k in 0..n {
            for j in 0..n {
                for i in 0..n {
                    let x = (i as f64).mul_add(step, center.x - size);
                    let y = (j as f64).mul_add(step, center.y - size);
                    let z = (k as f64).mul_add(step, center.z - size);
                    positions.push(Point3::new(x, y, z));
                }
            }
        }

        // Create tetrahedra (5 tets per cube cell)
        let mut tetrahedra = Vec::new();

        let idx = |i: usize, j: usize, k: usize| k * n * n + j * n + i;

        for k in 0..(n - 1) {
            for j in 0..(n - 1) {
                for i in 0..(n - 1) {
                    // 8 corners of the cube cell
                    let v000 = idx(i, j, k);
                    let v100 = idx(i + 1, j, k);
                    let v010 = idx(i, j + 1, k);
                    let v110 = idx(i + 1, j + 1, k);
                    let v001 = idx(i, j, k + 1);
                    let v101 = idx(i + 1, j, k + 1);
                    let v011 = idx(i, j + 1, k + 1);
                    let v111 = idx(i + 1, j + 1, k + 1);

                    // 5-tetrahedra decomposition
                    tetrahedra.push(Tetrahedron::new(v000, v100, v010, v001));
                    tetrahedra.push(Tetrahedron::new(v100, v110, v010, v111));
                    tetrahedra.push(Tetrahedron::new(v100, v001, v101, v111));
                    tetrahedra.push(Tetrahedron::new(v010, v001, v011, v111));
                    tetrahedra.push(Tetrahedron::new(v100, v010, v001, v111));
                }
            }
        }

        Self::from_tetrahedra(name, positions, tetrahedra, config)
    }

    /// Create a sphere soft body.
    ///
    /// # Arguments
    ///
    /// * `name` - Name
    /// * `center` - Center position
    /// * `radius` - Radius
    /// * `rings` - Number of latitude rings
    /// * `config` - Configuration
    #[must_use]
    pub fn sphere(
        name: &str,
        center: Point3<f64>,
        radius: f64,
        rings: usize,
        config: SoftBodyConfig,
    ) -> Self {
        // For simplicity, create a cube and tetrahedralize the sphere approximation
        // A proper implementation would use a spherical mesh generator
        let subdivisions = rings.max(1);
        let mut body = Self::cube(name, center, radius, subdivisions, config);

        // Move vertices to sphere surface
        for pos in &mut body.positions {
            let dir = *pos - center;
            let dist = dir.norm();
            if dist > 1e-10 {
                *pos = center + dir * (radius / dist);
            }
        }

        body
    }

    /// Get the configuration.
    #[must_use]
    pub const fn config(&self) -> &SoftBodyConfig {
        &self.config
    }

    /// Get the tetrahedra.
    #[must_use]
    pub fn tetrahedra(&self) -> &[Tetrahedron] {
        &self.tetrahedra
    }

    /// Get the number of tetrahedra.
    #[must_use]
    pub fn num_tetrahedra(&self) -> usize {
        self.tetrahedra.len()
    }

    /// Get the surface triangles.
    #[must_use]
    pub fn surface_triangles(&self) -> &[[usize; 3]] {
        &self.surface_triangles
    }

    /// Compute the total volume.
    #[must_use]
    pub fn volume(&self) -> f64 {
        self.tetrahedra
            .iter()
            .map(|tet| tet.volume(&self.positions))
            .sum()
    }

    /// Compute the rest volume.
    #[must_use]
    pub fn rest_volume(&self) -> f64 {
        self.constraints
            .iter()
            .filter_map(|c| {
                if let Constraint::Volume(vc) = c {
                    Some(vc.rest_volume)
                } else {
                    None
                }
            })
            .sum()
    }

    /// Pin the bottom vertices (vertices with minimum z).
    pub fn pin_bottom(&mut self, threshold: f64) {
        if self.positions.is_empty() {
            return;
        }

        let min_z = self
            .positions
            .iter()
            .map(|p| p.z)
            .fold(f64::INFINITY, f64::min);

        // Collect indices first to avoid borrow issues
        let to_pin: Vec<usize> = self
            .positions
            .iter()
            .enumerate()
            .filter(|(_, pos)| (pos.z - min_z).abs() < threshold)
            .map(|(i, _)| i)
            .collect();

        for i in to_pin {
            self.pin_vertex(i);
        }
    }

    /// Pin all surface vertices.
    pub fn pin_surface(&mut self) {
        // Collect indices first to avoid borrow issues
        let to_pin: Vec<usize> = self
            .surface_triangles
            .iter()
            .flat_map(|tri| tri.iter().copied())
            .collect();

        for v in to_pin {
            self.pin_vertex(v);
        }
    }
}

impl DeformableBody for SoftBody {
    fn id(&self) -> DeformableId {
        self.id
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn num_vertices(&self) -> usize {
        self.positions.len()
    }

    fn positions(&self) -> &[Point3<f64>] {
        &self.positions
    }

    fn positions_mut(&mut self) -> &mut [Point3<f64>] {
        &mut self.positions
    }

    fn velocities(&self) -> &[Vector3<f64>] {
        &self.velocities
    }

    fn velocities_mut(&mut self) -> &mut [Vector3<f64>] {
        &mut self.velocities
    }

    fn inverse_masses(&self) -> &[f64] {
        &self.inv_masses
    }

    fn vertex_flags(&self) -> &[VertexFlags] {
        &self.vertex_flags
    }

    fn vertex_flags_mut(&mut self) -> &mut [VertexFlags] {
        &mut self.vertex_flags
    }

    fn constraints(&self) -> &[Constraint] {
        &self.constraints
    }

    fn material(&self) -> &Material {
        &self.material
    }

    fn pin_vertex(&mut self, index: usize) {
        if index < self.inv_masses.len() {
            self.inv_masses[index] = 0.0;
            self.vertex_flags[index].insert(VertexFlags::PINNED);
        }
    }

    fn unpin_vertex(&mut self, index: usize) {
        if index < self.inv_masses.len() && index < self.masses.len() {
            self.inv_masses[index] = 1.0 / self.masses[index];
            self.vertex_flags[index].remove(VertexFlags::PINNED);
        }
    }

    fn is_pinned(&self, index: usize) -> bool {
        if index < self.vertex_flags.len() {
            self.vertex_flags[index].contains(VertexFlags::PINNED)
        } else {
            false
        }
    }

    fn apply_force(&mut self, index: usize, force: Vector3<f64>) {
        if index < self.external_forces.len() {
            self.external_forces[index] += force;
        }
    }

    fn external_forces(&self) -> &[Vector3<f64>] {
        &self.external_forces
    }

    fn clear_forces(&mut self) {
        for f in &mut self.external_forces {
            *f = Vector3::zeros();
        }
    }

    fn total_mass(&self) -> f64 {
        self.masses.iter().sum()
    }

    fn center_of_mass(&self) -> Point3<f64> {
        let total_mass = self.total_mass();
        if total_mass <= 0.0 {
            return Point3::origin();
        }

        let weighted_sum: Vector3<f64> = self
            .positions
            .iter()
            .zip(self.masses.iter())
            .map(|(p, &m)| p.coords * m)
            .sum();

        Point3::from(weighted_sum / total_mass)
    }

    fn bounding_box(&self) -> (Point3<f64>, Point3<f64>) {
        if self.positions.is_empty() {
            return (Point3::origin(), Point3::origin());
        }

        let mut min = self.positions[0];
        let mut max = self.positions[0];

        for p in &self.positions {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            min.z = min.z.min(p.z);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
            max.z = max.z.max(p.z);
        }

        (min, max)
    }

    fn clone_box(&self) -> Box<dyn DeformableBody + Send + Sync> {
        Box::new(self.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solver::{SolverConfig, XpbdSolver};

    #[test]
    fn test_soft_body_cube() {
        let body = SoftBody::cube(
            "test_cube",
            Point3::origin(),
            0.5,
            0,
            SoftBodyConfig::default(),
        );

        assert_eq!(body.num_vertices(), 8); // 2x2x2 for subdivisions=0
        assert!(body.num_tetrahedra() > 0);
        assert!(!body.surface_triangles().is_empty());
    }

    #[test]
    fn test_soft_body_volume() {
        let body = SoftBody::cube(
            "test_cube",
            Point3::origin(),
            0.5,
            0,
            SoftBodyConfig::default(),
        );

        let volume = body.volume();
        // Cube with half-size 0.5 has volume 1.0
        assert!(
            (volume - 1.0).abs() < 0.1,
            "Volume should be ~1.0, got {}",
            volume
        );
    }

    #[test]
    fn test_soft_body_simulation() {
        let mut body = SoftBody::cube(
            "jelly",
            Point3::new(0.0, 0.0, 1.0),
            0.3,
            1,
            SoftBodyConfig::gelatin(),
        );

        // Pin bottom
        body.pin_bottom(0.1);

        let mut solver = XpbdSolver::new(SolverConfig {
            num_iterations: 10,
            ..SolverConfig::default()
        });
        let gravity = Vector3::new(0.0, 0.0, -9.81);

        let initial_com = body.center_of_mass();

        // Simulate
        for _ in 0..60 {
            solver.step(&mut body, gravity, 1.0 / 60.0);
        }

        let final_com = body.center_of_mass();

        // Center of mass should have moved down
        assert!(
            final_com.z < initial_com.z,
            "COM should have dropped: {} -> {}",
            initial_com.z,
            final_com.z
        );
    }

    #[test]
    fn test_soft_body_configs() {
        let rubber = SoftBodyConfig::rubber();
        let gelatin = SoftBodyConfig::gelatin();
        let foam = SoftBodyConfig::foam();

        // Foam should be much lighter
        assert!(foam.density < gelatin.density);
        assert!(foam.density < rubber.density);

        // Rubber should be nearly incompressible
        assert!(rubber.volume_compliance < foam.volume_compliance);
    }

    #[test]
    fn test_soft_body_from_tetrahedra() {
        // Create a single tetrahedron
        let positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];
        let tetrahedra = vec![Tetrahedron::new(0, 1, 2, 3)];

        let body =
            SoftBody::from_tetrahedra("tet", positions, tetrahedra, SoftBodyConfig::default());

        assert_eq!(body.num_vertices(), 4);
        assert_eq!(body.num_tetrahedra(), 1);
        assert_eq!(body.surface_triangles().len(), 4); // 4 faces

        // Should have 6 distance constraints + 1 volume constraint = 7
        assert_eq!(body.constraints().len(), 7);
    }

    #[test]
    fn test_soft_body_surface_extraction() {
        // Cube with subdivisions=0 should have 12 surface triangles (2 per face)
        let body = SoftBody::cube(
            "test_cube",
            Point3::origin(),
            0.5,
            0,
            SoftBodyConfig::default(),
        );

        assert_eq!(body.surface_triangles().len(), 12);
    }

    #[test]
    fn test_soft_body_pin() {
        let mut body = SoftBody::cube(
            "test_cube",
            Point3::origin(),
            0.5,
            0,
            SoftBodyConfig::default(),
        );

        body.pin_vertex(0);
        assert!(body.is_pinned(0));

        body.unpin_vertex(0);
        assert!(!body.is_pinned(0));
    }
}
