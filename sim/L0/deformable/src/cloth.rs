//! 2D deformable bodies: cloth and membrane simulation.
//!
//! Cloth is represented as a triangle mesh with distance and bending constraints.
//!
//! # Structure
//!
//! A cloth mesh consists of:
//!
//! - **Vertices**: Particle positions
//! - **Distance constraints**: Along edges to resist stretching
//! - **Bending constraints**: Along shared edges to resist folding
//!
//! ```text
//!   ●───●───●
//!   |\  |\  |
//!   | \ | \ |
//!   ●───●───●
//!   |\  |\  |
//!   | \ | \ |
//!   ●───●───●
//! ```

use hashbrown::HashSet;
use nalgebra::{Point3, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::DeformableBody;
use crate::constraints::{BendingConstraint, Constraint, DistanceConstraint};
use crate::material::{Material, MaterialPreset};
use crate::mesh::Triangle;
use crate::types::{DeformableId, VertexFlags, next_deformable_id};

/// Configuration for cloth simulation.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ClothConfig {
    /// Mass per unit area (kg/m²).
    pub area_density: f64,
    /// Distance compliance (0 = inextensible).
    pub distance_compliance: f64,
    /// Bending compliance (0 = rigid, larger = more flexible).
    pub bending_compliance: f64,
    /// Whether to add bending constraints.
    pub enable_bending: bool,
    /// Damping coefficient.
    pub damping: f64,
    /// Thickness for collision detection.
    pub thickness: f64,
}

impl Default for ClothConfig {
    fn default() -> Self {
        Self {
            area_density: 0.3,         // Light fabric
            distance_compliance: 0.0,  // Inextensible
            bending_compliance: 0.001, // Flexible
            enable_bending: true,
            damping: 0.02,
            thickness: 0.001, // 1 mm
        }
    }
}

impl ClothConfig {
    /// Create a config for cotton fabric.
    #[must_use]
    pub const fn cotton() -> Self {
        Self {
            area_density: 0.2,
            distance_compliance: 1e-6,
            bending_compliance: 0.01,
            enable_bending: true,
            damping: 0.05,
            thickness: 0.001,
        }
    }

    /// Create a config for silk fabric (light, very flexible).
    #[must_use]
    pub const fn silk() -> Self {
        Self {
            area_density: 0.05,
            distance_compliance: 1e-5,
            bending_compliance: 0.1,
            enable_bending: true,
            damping: 0.03,
            thickness: 0.0005,
        }
    }

    /// Create a config for leather (heavy, less flexible).
    #[must_use]
    pub const fn leather() -> Self {
        Self {
            area_density: 1.0,
            distance_compliance: 1e-7,
            bending_compliance: 0.0001,
            enable_bending: true,
            damping: 0.02,
            thickness: 0.003,
        }
    }

    /// Create a config for rubber sheet.
    #[must_use]
    pub const fn rubber() -> Self {
        Self {
            area_density: 1.5,
            distance_compliance: 1e-5,
            bending_compliance: 0.001,
            enable_bending: true,
            damping: 0.1,
            thickness: 0.002,
        }
    }

    /// Create a config for paper.
    #[must_use]
    pub const fn paper() -> Self {
        Self {
            area_density: 0.08,
            distance_compliance: 1e-8,
            bending_compliance: 0.0001, // Easy to fold but holds shape
            enable_bending: true,
            damping: 0.1,
            thickness: 0.0001,
        }
    }

    /// Create a config for membrane (like a balloon).
    #[must_use]
    pub const fn membrane() -> Self {
        Self {
            area_density: 0.1,
            distance_compliance: 1e-5,
            bending_compliance: 1.0, // Almost no bending resistance
            enable_bending: false,
            damping: 0.05,
            thickness: 0.0005,
        }
    }
}

/// A 2D deformable body for cloth and membrane simulation.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Cloth {
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
    /// Triangles.
    triangles: Vec<Triangle>,
    /// Material.
    material: Material,
    /// Configuration.
    config: ClothConfig,
    /// Grid dimensions (for rectangular cloth).
    grid_size: Option<(usize, usize)>,
}

impl Cloth {
    /// Create a rectangular cloth grid.
    ///
    /// # Arguments
    ///
    /// * `name` - Name of the cloth
    /// * `origin` - Position of the (0,0) corner
    /// * `u_axis` - Direction and size of the U axis
    /// * `v_axis` - Direction and size of the V axis
    /// * `u_segments` - Number of segments along U
    /// * `v_segments` - Number of segments along V
    /// * `config` - Configuration
    #[must_use]
    pub fn grid(
        name: &str,
        origin: Point3<f64>,
        u_axis: Vector3<f64>,
        v_axis: Vector3<f64>,
        u_segments: usize,
        v_segments: usize,
        config: ClothConfig,
    ) -> Self {
        let u_segments = u_segments.max(1);
        let v_segments = v_segments.max(1);
        let u_verts = u_segments + 1;
        let v_verts = v_segments + 1;
        let num_vertices = u_verts * v_verts;

        // Create vertices
        let mut positions = Vec::with_capacity(num_vertices);
        for j in 0..v_verts {
            for i in 0..u_verts {
                let u = i as f64 / u_segments as f64;
                let v = j as f64 / v_segments as f64;
                positions.push(origin + u_axis * u + v_axis * v);
            }
        }

        // Create triangles
        let mut triangles = Vec::with_capacity(2 * u_segments * v_segments);
        for j in 0..v_segments {
            for i in 0..u_segments {
                let idx00 = j * u_verts + i;
                let idx10 = j * u_verts + i + 1;
                let idx01 = (j + 1) * u_verts + i;
                let idx11 = (j + 1) * u_verts + i + 1;

                // Two triangles per quad
                triangles.push(Triangle::new(idx00, idx10, idx11));
                triangles.push(Triangle::new(idx00, idx11, idx01));
            }
        }

        Self::from_triangles(name, positions, triangles, config).with_grid_size(u_verts, v_verts)
    }

    /// Create a cloth from vertices and triangles.
    #[must_use]
    pub fn from_triangles(
        name: &str,
        positions: Vec<Point3<f64>>,
        triangles: Vec<Triangle>,
        config: ClothConfig,
    ) -> Self {
        let num_vertices = positions.len();

        // Compute masses based on triangle areas
        let mut masses = vec![0.0; num_vertices];
        for tri in &triangles {
            let area = tri.area(&positions);
            let mass_per_vertex = (config.area_density * area) / 3.0;

            for &v in &tri.vertices {
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

        // Distance constraints for edges
        let mut edges_added: HashSet<(usize, usize)> = HashSet::new();
        for tri in &triangles {
            for edge in tri.edges() {
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

        // Bending constraints for shared edges
        if config.enable_bending {
            // Find triangle pairs sharing an edge
            let mut edge_triangles: hashbrown::HashMap<(usize, usize), Vec<usize>> =
                hashbrown::HashMap::new();

            for (tri_idx, tri) in triangles.iter().enumerate() {
                for edge in tri.edges() {
                    let key = if edge.v0 < edge.v1 {
                        (edge.v0, edge.v1)
                    } else {
                        (edge.v1, edge.v0)
                    };
                    edge_triangles.entry(key).or_default().push(tri_idx);
                }
            }

            // Create bending constraints for edges shared by exactly 2 triangles
            for ((v0, v1), tris) in edge_triangles {
                if tris.len() == 2 {
                    let tri0 = &triangles[tris[0]];
                    let tri1 = &triangles[tris[1]];

                    // Find the opposite vertices
                    let v2 = tri0.vertices.iter().find(|&&v| v != v0 && v != v1).copied();
                    let v3 = tri1.vertices.iter().find(|&&v| v != v0 && v != v1).copied();

                    if let (Some(v2), Some(v3)) = (v2, v3) {
                        constraints.push(Constraint::Bending(
                            BendingConstraint::dihedral_from_positions(
                                v0,
                                v1,
                                v2,
                                v3,
                                &positions,
                                config.bending_compliance,
                            ),
                        ));
                    }
                }
            }
        }

        let material = Material::preset(MaterialPreset::Cloth).with_damping(config.damping);

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
            triangles,
            material,
            config,
            grid_size: None,
        }
    }

    /// Set the grid size (for rectangular cloth).
    const fn with_grid_size(mut self, u_verts: usize, v_verts: usize) -> Self {
        self.grid_size = Some((u_verts, v_verts));
        self
    }

    /// Get the configuration.
    #[must_use]
    pub const fn config(&self) -> &ClothConfig {
        &self.config
    }

    /// Get the triangles.
    #[must_use]
    pub fn triangles(&self) -> &[Triangle] {
        &self.triangles
    }

    /// Get the number of triangles.
    #[must_use]
    pub fn num_triangles(&self) -> usize {
        self.triangles.len()
    }

    /// Get the grid size if this is a rectangular cloth.
    #[must_use]
    pub const fn grid_size(&self) -> Option<(usize, usize)> {
        self.grid_size
    }

    /// Pin an edge of a grid cloth.
    ///
    /// # Arguments
    ///
    /// * `edge` - Which edge to pin: "top", "bottom", "left", "right"
    pub fn pin_edge(&mut self, edge: &str) {
        if let Some((u_verts, v_verts)) = self.grid_size {
            match edge.to_lowercase().as_str() {
                "top" | "back" => {
                    // Last row
                    for i in 0..u_verts {
                        let idx = (v_verts - 1) * u_verts + i;
                        self.pin_vertex(idx);
                    }
                }
                "bottom" | "front" => {
                    // First row
                    for i in 0..u_verts {
                        self.pin_vertex(i);
                    }
                }
                "left" => {
                    // First column
                    for j in 0..v_verts {
                        let idx = j * u_verts;
                        self.pin_vertex(idx);
                    }
                }
                "right" => {
                    // Last column
                    for j in 0..v_verts {
                        let idx = j * u_verts + (u_verts - 1);
                        self.pin_vertex(idx);
                    }
                }
                _ => {}
            }
        }
    }

    /// Pin corners of a grid cloth.
    pub fn pin_corners(&mut self) {
        if let Some((u_verts, v_verts)) = self.grid_size {
            self.pin_vertex(0); // (0, 0)
            self.pin_vertex(u_verts - 1); // (u_max, 0)
            self.pin_vertex((v_verts - 1) * u_verts); // (0, v_max)
            self.pin_vertex(v_verts * u_verts - 1); // (u_max, v_max)
        }
    }

    /// Get the vertex index for grid coordinates (for grid cloth).
    #[must_use]
    pub fn grid_vertex(&self, u: usize, v: usize) -> Option<usize> {
        self.grid_size.map(|(u_verts, v_verts)| {
            let u = u.min(u_verts - 1);
            let v = v.min(v_verts - 1);
            v * u_verts + u
        })
    }

    /// Compute the total surface area.
    #[must_use]
    pub fn surface_area(&self) -> f64 {
        self.triangles.iter().map(|t| t.area(&self.positions)).sum()
    }

    /// Compute the surface normal at a vertex (averaged from adjacent triangles).
    #[must_use]
    pub fn vertex_normal(&self, vertex: usize) -> Vector3<f64> {
        let mut normal = Vector3::zeros();
        let mut count = 0;

        for tri in &self.triangles {
            if tri.contains(vertex) {
                normal += tri.normal(&self.positions);
                count += 1;
            }
        }

        if count > 0 {
            let n = normal / f64::from(count);
            let len = n.norm();
            if len > 1e-10 {
                return n / len;
            }
        }

        Vector3::z()
    }

    /// Apply wind force to the cloth.
    ///
    /// # Arguments
    ///
    /// * `wind_velocity` - Wind velocity vector
    /// * `drag_coefficient` - Drag coefficient (typically 1.0-2.0)
    pub fn apply_wind(&mut self, wind_velocity: Vector3<f64>, drag_coefficient: f64) {
        let air_density = 1.225; // kg/m³

        for tri in &self.triangles {
            let normal = tri.normal(&self.positions);
            let area = tri.area(&self.positions);

            // Relative velocity (wind - cloth velocity at triangle center)
            let center_vel = (self.velocities[tri.vertices[0]]
                + self.velocities[tri.vertices[1]]
                + self.velocities[tri.vertices[2]])
                / 3.0;
            let rel_vel = wind_velocity - center_vel;

            // Project onto normal (only normal component creates drag)
            let normal_speed = rel_vel.dot(&normal);

            // Drag force: F = 0.5 * rho * v² * Cd * A * sign(v)
            let drag_magnitude =
                0.5 * air_density * normal_speed.abs() * normal_speed * drag_coefficient * area;

            let drag_force = normal * drag_magnitude;

            // Distribute to vertices
            for &v in &tri.vertices {
                self.external_forces[v] += drag_force / 3.0;
            }
        }
    }
}

impl DeformableBody for Cloth {
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

        let t = self.config.thickness;
        let mut min = self.positions[0] - Vector3::new(t, t, t);
        let mut max = self.positions[0] + Vector3::new(t, t, t);

        for p in &self.positions {
            min.x = min.x.min(p.x - t);
            min.y = min.y.min(p.y - t);
            min.z = min.z.min(p.z - t);
            max.x = max.x.max(p.x + t);
            max.y = max.y.max(p.y + t);
            max.z = max.z.max(p.z + t);
        }

        (min, max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solver::{SolverConfig, XpbdSolver};

    #[test]
    fn test_cloth_grid() {
        let cloth = Cloth::grid(
            "test",
            Point3::origin(),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            4,
            4,
            ClothConfig::default(),
        );

        assert_eq!(cloth.num_vertices(), 25); // 5x5
        assert_eq!(cloth.num_triangles(), 32); // 4x4 quads * 2 triangles
        assert_eq!(cloth.grid_size(), Some((5, 5)));
    }

    #[test]
    fn test_cloth_pin_edge() {
        let mut cloth = Cloth::grid(
            "test",
            Point3::origin(),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            4,
            4,
            ClothConfig::default(),
        );

        cloth.pin_edge("top");

        // Check that top edge is pinned
        for i in 0..5 {
            let idx = 4 * 5 + i; // Top row
            assert!(cloth.is_pinned(idx), "Vertex {} should be pinned", idx);
        }
    }

    #[test]
    fn test_cloth_grid_vertex() {
        let cloth = Cloth::grid(
            "test",
            Point3::origin(),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            4,
            4,
            ClothConfig::default(),
        );

        assert_eq!(cloth.grid_vertex(0, 0), Some(0));
        assert_eq!(cloth.grid_vertex(4, 0), Some(4));
        assert_eq!(cloth.grid_vertex(0, 4), Some(20));
        assert_eq!(cloth.grid_vertex(4, 4), Some(24));
    }

    #[test]
    fn test_cloth_surface_area() {
        let cloth = Cloth::grid(
            "test",
            Point3::origin(),
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(0.0, 2.0, 0.0),
            1,
            1,
            ClothConfig::default(),
        );

        let area = cloth.surface_area();
        assert!(
            (area - 4.0).abs() < 1e-10,
            "Area should be 4.0, got {}",
            area
        );
    }

    #[test]
    fn test_cloth_simulation() {
        let mut cloth = Cloth::grid(
            "flag",
            Point3::new(0.0, 0.0, 1.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, -1.0),
            10,
            10,
            ClothConfig::cotton(),
        );

        // Pin the left edge
        cloth.pin_edge("left");

        let mut solver = XpbdSolver::new(SolverConfig::default());
        let gravity = Vector3::new(0.0, 0.0, -9.81);

        // Simulate
        for _ in 0..60 {
            solver.step(&mut cloth, gravity, 1.0 / 60.0);
        }

        // The right side should have fallen
        let right_bottom_idx = cloth.grid_vertex(10, 10).unwrap_or(0);
        assert!(
            cloth.positions()[right_bottom_idx].z < 0.0,
            "Right side should have fallen"
        );

        // The left side should still be pinned
        let left_top_idx = cloth.grid_vertex(0, 0).unwrap_or(0);
        assert!(
            (cloth.positions()[left_top_idx].z - 1.0).abs() < 0.01,
            "Left top should be pinned at z=1"
        );
    }

    #[test]
    fn test_cloth_configs() {
        let cotton = ClothConfig::cotton();
        let silk = ClothConfig::silk();
        let leather = ClothConfig::leather();

        // Silk should be lighter
        assert!(silk.area_density < cotton.area_density);

        // Leather should be heavier and stiffer
        assert!(leather.area_density > cotton.area_density);
        assert!(leather.bending_compliance < cotton.bending_compliance);
    }

    #[test]
    fn test_cloth_from_triangles() {
        let positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
        ];
        let triangles = vec![Triangle::new(0, 1, 2)];

        let cloth = Cloth::from_triangles("triangle", positions, triangles, ClothConfig::default());

        assert_eq!(cloth.num_vertices(), 3);
        assert_eq!(cloth.num_triangles(), 1);
        assert!(cloth.constraints().len() >= 3); // At least 3 distance constraints
    }

    #[test]
    fn test_cloth_vertex_normal() {
        let cloth = Cloth::grid(
            "test",
            Point3::origin(),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            1,
            1,
            ClothConfig::default(),
        );

        // Center vertex should have normal pointing up (z)
        let normal = cloth.vertex_normal(0);
        assert!(normal.z.abs() > 0.9, "Normal should point in z direction");
    }
}
