//! 1D deformable bodies: capsule chains for ropes, cables, and hair.
//!
//! A capsule chain is a sequence of particles connected by distance constraints,
//! forming a rope-like structure. Each segment can optionally have a radius for
//! collision detection.
//!
//! # Structure
//!
//! ```text
//! ●───●───●───●───●
//! 0   1   2   3   4
//!  \   \   \   \
//!   d0  d1  d2  d3   (distance constraints)
//! ```
//!
//! Bending constraints can be added to resist sharp bends:
//!
//! ```text
//! ●───●───●
//!  \  |  /
//!   \ | /
//!    \|/
//!     bend constraint at vertex 1
//! ```

use nalgebra::{Point3, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::DeformableBody;
use crate::constraints::{BendingConstraint, Constraint, DistanceConstraint};
use crate::material::{Material, MaterialPreset};
use crate::types::{DeformableId, VertexFlags, next_deformable_id};

/// Configuration for a capsule chain.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CapsuleChainConfig {
    /// Radius of each capsule segment (for collision).
    pub radius: f64,
    /// Mass per unit length (kg/m).
    pub linear_density: f64,
    /// Distance compliance (0 = rigid stretch).
    pub distance_compliance: f64,
    /// Bending compliance (0 = rigid bend, `f64::INFINITY` = no bending resistance).
    pub bending_compliance: f64,
    /// Whether to add bending constraints.
    pub enable_bending: bool,
    /// Damping coefficient.
    pub damping: f64,
}

impl Default for CapsuleChainConfig {
    fn default() -> Self {
        Self {
            radius: 0.01,              // 1 cm
            linear_density: 1.0,       // 1 kg/m
            distance_compliance: 0.0,  // Inextensible
            bending_compliance: 0.001, // Flexible
            enable_bending: true,
            damping: 0.02,
        }
    }
}

impl CapsuleChainConfig {
    /// Create a config for a rope (flexible, some stretch).
    #[must_use]
    pub const fn rope(radius: f64) -> Self {
        Self {
            radius,
            linear_density: 0.5,
            distance_compliance: 1e-6,
            bending_compliance: 0.01,
            enable_bending: true,
            damping: 0.05,
        }
    }

    /// Create a config for a steel cable (stiff, minimal stretch).
    #[must_use]
    pub fn steel_cable(radius: f64) -> Self {
        Self {
            radius,
            linear_density: 7.8 * std::f64::consts::PI * radius * radius, // Steel density
            distance_compliance: 1e-10,                                   // Very stiff
            bending_compliance: 1e-4,                                     // Some bending resistance
            enable_bending: true,
            damping: 0.01,
        }
    }

    /// Create a config for a soft cable (flexible, stretchable).
    #[must_use]
    pub const fn soft_cable(radius: f64) -> Self {
        Self {
            radius,
            linear_density: 0.2,
            distance_compliance: 1e-4,
            bending_compliance: 0.1,
            enable_bending: true,
            damping: 0.1,
        }
    }

    /// Create a config for hair strand (very flexible).
    #[must_use]
    pub const fn hair(radius: f64) -> Self {
        Self {
            radius,
            linear_density: 0.01,
            distance_compliance: 1e-5,
            bending_compliance: 1.0, // Very flexible bending
            enable_bending: true,
            damping: 0.2,
        }
    }

    /// Create a config for a chain (rigid links).
    #[must_use]
    pub const fn chain(link_radius: f64) -> Self {
        Self {
            radius: link_radius,
            linear_density: 3.0, // Heavy metal links
            distance_compliance: 0.0,
            bending_compliance: f64::INFINITY, // No bending resistance (free hinges)
            enable_bending: false,
            damping: 0.02,
        }
    }
}

/// A 1D deformable body consisting of connected capsules.
///
/// Used for simulating ropes, cables, chains, and hair.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CapsuleChain {
    /// Unique identifier.
    id: DeformableId,
    /// Name of this chain.
    name: String,
    /// Vertex positions.
    positions: Vec<Point3<f64>>,
    /// Vertex velocities.
    velocities: Vec<Vector3<f64>>,
    /// Inverse masses (0 for pinned).
    inv_masses: Vec<f64>,
    /// Original masses (for unpinning).
    masses: Vec<f64>,
    /// Vertex flags.
    vertex_flags: Vec<VertexFlags>,
    /// External forces accumulated this frame.
    external_forces: Vec<Vector3<f64>>,
    /// Constraints.
    constraints: Vec<Constraint>,
    /// Material properties.
    material: Material,
    /// Configuration.
    config: CapsuleChainConfig,
    /// Rest lengths between consecutive vertices.
    rest_lengths: Vec<f64>,
}

impl CapsuleChain {
    /// Create a new capsule chain between two points.
    ///
    /// # Arguments
    ///
    /// * `name` - Name of the chain
    /// * `start` - Starting position
    /// * `end` - Ending position
    /// * `num_segments` - Number of segments (vertices = segments + 1)
    /// * `config` - Configuration
    #[must_use]
    pub fn new(
        name: &str,
        start: Point3<f64>,
        end: Point3<f64>,
        num_segments: usize,
        config: CapsuleChainConfig,
    ) -> Self {
        let num_segments = num_segments.max(1);
        let num_vertices = num_segments + 1;

        // Create evenly spaced vertices
        let direction = end - start;
        let total_length = direction.norm();
        let segment_length = total_length / num_segments as f64;

        let mut positions = Vec::with_capacity(num_vertices);
        for i in 0..num_vertices {
            let t = i as f64 / num_segments as f64;
            positions.push(start + direction * t);
        }

        // Compute masses from linear density
        let mass_per_vertex = config.linear_density * segment_length;
        // End vertices get half mass
        let mut masses = vec![mass_per_vertex; num_vertices];
        masses[0] *= 0.5;
        masses[num_vertices - 1] *= 0.5;

        let inv_masses: Vec<f64> = masses.iter().map(|&m| 1.0 / m).collect();

        // Create constraints
        let mut constraints = Vec::new();
        let mut rest_lengths = Vec::with_capacity(num_segments);

        // Distance constraints
        for i in 0..num_segments {
            rest_lengths.push(segment_length);
            constraints.push(Constraint::Distance(DistanceConstraint::new(
                i,
                i + 1,
                segment_length,
                config.distance_compliance,
            )));
        }

        // Bending constraints
        if config.enable_bending && config.bending_compliance.is_finite() {
            for i in 0..(num_vertices.saturating_sub(2)) {
                // Rest angle is 180 degrees (straight)
                constraints.push(Constraint::Bending(BendingConstraint::chain(
                    i,
                    i + 1,
                    i + 2,
                    std::f64::consts::PI,
                    config.bending_compliance,
                )));
            }
        }

        // Create material from config
        let material = Material::preset(MaterialPreset::Rope).with_damping(config.damping);

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
            material,
            config,
            rest_lengths,
        }
    }

    /// Create a capsule chain from a set of points.
    ///
    /// The chain follows the provided waypoints.
    #[must_use]
    pub fn from_points(name: &str, points: &[Point3<f64>], config: CapsuleChainConfig) -> Self {
        if points.len() < 2 {
            return Self::new(
                name,
                points.first().copied().unwrap_or_else(Point3::origin),
                points.last().copied().unwrap_or_else(Point3::origin),
                1,
                config,
            );
        }

        let num_vertices = points.len();
        let positions: Vec<_> = points.to_vec();

        // Compute segment lengths
        let mut rest_lengths = Vec::with_capacity(num_vertices - 1);
        for i in 0..(num_vertices - 1) {
            rest_lengths.push((positions[i + 1] - positions[i]).norm());
        }

        // Compute masses
        let total_length: f64 = rest_lengths.iter().sum();
        let avg_segment_length = total_length / (num_vertices - 1) as f64;
        let mass_per_vertex = config.linear_density * avg_segment_length;

        let mut masses = vec![mass_per_vertex; num_vertices];
        masses[0] *= 0.5;
        masses[num_vertices - 1] *= 0.5;

        let inv_masses: Vec<f64> = masses.iter().map(|&m| 1.0 / m).collect();

        // Create constraints
        let mut constraints = Vec::new();

        // Distance constraints
        for (i, &rest_len) in rest_lengths.iter().enumerate() {
            constraints.push(Constraint::Distance(DistanceConstraint::new(
                i,
                i + 1,
                rest_len,
                config.distance_compliance,
            )));
        }

        // Bending constraints
        if config.enable_bending && config.bending_compliance.is_finite() {
            for i in 0..(num_vertices.saturating_sub(2)) {
                let rest_angle = {
                    let e1 = positions[i] - positions[i + 1];
                    let e2 = positions[i + 2] - positions[i + 1];
                    e1.angle(&e2)
                };
                constraints.push(Constraint::Bending(BendingConstraint::chain(
                    i,
                    i + 1,
                    i + 2,
                    rest_angle,
                    config.bending_compliance,
                )));
            }
        }

        let material = Material::preset(MaterialPreset::Rope).with_damping(config.damping);

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
            material,
            config,
            rest_lengths,
        }
    }

    /// Get the configuration.
    #[must_use]
    pub const fn config(&self) -> &CapsuleChainConfig {
        &self.config
    }

    /// Get the radius of the chain.
    #[must_use]
    pub const fn radius(&self) -> f64 {
        self.config.radius
    }

    /// Get the number of segments.
    #[must_use]
    pub fn num_segments(&self) -> usize {
        self.positions.len().saturating_sub(1)
    }

    /// Get the total length of the chain (sum of current segment lengths).
    #[must_use]
    pub fn current_length(&self) -> f64 {
        let mut length = 0.0;
        for i in 0..self.num_segments() {
            length += (self.positions[i + 1] - self.positions[i]).norm();
        }
        length
    }

    /// Get the rest length of the chain.
    #[must_use]
    pub fn rest_length(&self) -> f64 {
        self.rest_lengths.iter().sum()
    }

    /// Get the start position.
    #[must_use]
    pub fn start_position(&self) -> Point3<f64> {
        self.positions
            .first()
            .copied()
            .unwrap_or_else(Point3::origin)
    }

    /// Get the end position.
    #[must_use]
    pub fn end_position(&self) -> Point3<f64> {
        self.positions
            .last()
            .copied()
            .unwrap_or_else(Point3::origin)
    }

    /// Set the position of a specific vertex.
    pub fn set_vertex_position(&mut self, index: usize, position: Point3<f64>) {
        if index < self.positions.len() {
            self.positions[index] = position;
        }
    }

    /// Get the position along the chain at parameter t (0 = start, 1 = end).
    #[must_use]
    pub fn position_at(&self, t: f64) -> Point3<f64> {
        let t = t.clamp(0.0, 1.0);
        let n = self.positions.len();

        if n < 2 {
            return self.start_position();
        }

        let segment_t = t * (n - 1) as f64;
        let segment_idx = (segment_t.floor() as usize).min(n - 2);
        let local_t = segment_t - segment_idx as f64;

        let p0 = self.positions[segment_idx];
        let p1 = self.positions[segment_idx + 1];

        p0 + (p1 - p0) * local_t
    }

    /// Get the tangent direction along the chain at parameter t.
    #[must_use]
    pub fn tangent_at(&self, t: f64) -> Vector3<f64> {
        let t = t.clamp(0.0, 1.0);
        let n = self.positions.len();

        if n < 2 {
            return Vector3::z();
        }

        let segment_t = t * (n - 1) as f64;
        let segment_idx = (segment_t.floor() as usize).min(n - 2);

        let p0 = self.positions[segment_idx];
        let p1 = self.positions[segment_idx + 1];

        let dir = p1 - p0;
        let len = dir.norm();
        if len > 1e-10 { dir / len } else { Vector3::z() }
    }

    /// Apply a distributed force along the chain (like wind or drag).
    pub fn apply_distributed_force(&mut self, force_per_length: Vector3<f64>) {
        for i in 0..self.positions.len() {
            // Each vertex gets force proportional to segment lengths it's part of
            let mut segment_length = 0.0;
            if i > 0 {
                segment_length += self.rest_lengths[i - 1] * 0.5;
            }
            if i < self.rest_lengths.len() {
                segment_length += self.rest_lengths[i] * 0.5;
            }

            self.external_forces[i] += force_per_length * segment_length;
        }
    }
}

impl DeformableBody for CapsuleChain {
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

        let r = self.config.radius;
        let mut min = self.positions[0] - Vector3::new(r, r, r);
        let mut max = self.positions[0] + Vector3::new(r, r, r);

        for p in &self.positions {
            min.x = min.x.min(p.x - r);
            min.y = min.y.min(p.y - r);
            min.z = min.z.min(p.z - r);
            max.x = max.x.max(p.x + r);
            max.y = max.y.max(p.y + r);
            max.z = max.z.max(p.z + r);
        }

        (min, max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solver::{SolverConfig, XpbdSolver};

    #[test]
    fn test_capsule_chain_new() {
        let chain = CapsuleChain::new(
            "test",
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            4,
            CapsuleChainConfig::default(),
        );

        assert_eq!(chain.num_vertices(), 5);
        assert_eq!(chain.num_segments(), 4);
        assert_eq!(chain.name(), "test");
    }

    #[test]
    fn test_capsule_chain_positions() {
        let chain = CapsuleChain::new(
            "test",
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            4,
            CapsuleChainConfig::default(),
        );

        let positions = chain.positions();
        assert!((positions[0].x - 0.0).abs() < 1e-10);
        assert!((positions[1].x - 1.0).abs() < 1e-10);
        assert!((positions[2].x - 2.0).abs() < 1e-10);
        assert!((positions[3].x - 3.0).abs() < 1e-10);
        assert!((positions[4].x - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_capsule_chain_rest_length() {
        let chain = CapsuleChain::new(
            "test",
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            5,
            CapsuleChainConfig::default(),
        );

        assert!((chain.rest_length() - 5.0).abs() < 1e-10);
        assert!((chain.current_length() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_capsule_chain_from_points() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ];

        let chain = CapsuleChain::from_points("test", &points, CapsuleChainConfig::default());

        assert_eq!(chain.num_vertices(), 4);
        assert_eq!(chain.num_segments(), 3);
        assert!((chain.rest_length() - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_capsule_chain_position_at() {
        let chain = CapsuleChain::new(
            "test",
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            4,
            CapsuleChainConfig::default(),
        );

        let p0 = chain.position_at(0.0);
        let p_mid = chain.position_at(0.5);
        let p1 = chain.position_at(1.0);

        assert!((p0.x - 0.0).abs() < 1e-10);
        assert!((p_mid.x - 2.0).abs() < 1e-10);
        assert!((p1.x - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_capsule_chain_pin() {
        let mut chain = CapsuleChain::new(
            "test",
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            4,
            CapsuleChainConfig::default(),
        );

        chain.pin_vertex(0);
        assert!(chain.is_pinned(0));
        assert!(!chain.is_pinned(1));

        chain.unpin_vertex(0);
        assert!(!chain.is_pinned(0));
    }

    #[test]
    fn test_capsule_chain_simulation() {
        let mut chain = CapsuleChain::new(
            "rope",
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(2.0, 0.0, 2.0),
            10,
            CapsuleChainConfig::rope(0.01),
        );

        // Pin start
        chain.pin_vertex(0);

        let mut solver = XpbdSolver::new(SolverConfig::default());
        let gravity = Vector3::new(0.0, 0.0, -9.81);

        // Simulate for a bit
        for _ in 0..60 {
            solver.step(&mut chain, gravity, 1.0 / 60.0);
        }

        // The end should have fallen
        assert!(
            chain.end_position().z < 2.0,
            "End should have fallen: {}",
            chain.end_position().z
        );

        // The start should be pinned
        assert!(
            (chain.start_position().z - 2.0).abs() < 1e-10,
            "Start should be pinned: {}",
            chain.start_position().z
        );
    }

    #[test]
    fn test_capsule_chain_configs() {
        let rope = CapsuleChainConfig::rope(0.01);
        let steel = CapsuleChainConfig::steel_cable(0.005);
        let hair = CapsuleChainConfig::hair(0.001);

        // Steel should be stiffer
        assert!(steel.distance_compliance < rope.distance_compliance);
        assert!(steel.bending_compliance < rope.bending_compliance);

        // Hair should be more flexible than rope
        assert!(hair.bending_compliance > rope.bending_compliance);
    }

    #[test]
    fn test_capsule_chain_distributed_force() {
        let mut chain = CapsuleChain::new(
            "test",
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            4,
            CapsuleChainConfig::default(),
        );

        let wind = Vector3::new(0.0, 1.0, 0.0);
        chain.apply_distributed_force(wind);

        // All non-end vertices should have some force
        for f in chain.external_forces() {
            // Force should be in y direction
            assert!(f.y >= 0.0);
        }
    }
}
